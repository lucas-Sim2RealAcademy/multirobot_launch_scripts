// airsim_realsense_node.cpp — Q9 of GAP-CLOSURE-PLAN.md
//
// Per-drone C++ sensor node that impersonates realsense-ros + realsense_splitter on top of
// a Cosys-AirSim vehicle.  It replaces the Python RPC sensor loops in cuvslam_sim_bridge.py
// (stereo ~2.4 Hz, IMU ~22-30 Hz, depth ~8 Hz) with a native client that hits field rates
// (stereo >=30 Hz, IMU ~200 Hz, depth >=30 Hz).
//
// Design notes (each of these is load-bearing, do not "simplify" them away):
//
//  * SEPARATE RPC CLIENT PER STREAM.  AirSim's rpclib server serves each TCP connection on
//    its own worker thread; the per-call latency floor (~30 ms for a blocking render) is
//    therefore PER CONNECTION.  Three connections (stereo / depth / imu) is what makes
//    30 Hz stereo + 30 Hz depth + 200 Hz IMU possible at all.  It also requires the
//    SimModeBase.cpp worker-pool patch (spawned_actors_.Num()*4+8), or 4x stereo + 4x depth
//    blocking renders saturate the pool and starve the IMU connections.
//
//  * BOTH stereo images AND both camera_infos are stamped from responses[0].time_stamp.
//    simGetImages() returns per-response stamps that are NOT guaranteed identical across a
//    batch; cuVSLAM requires an exactly-matched stereo pair or it drops the frame.
//
//  * The Scene cameras are requested UNCOMPRESSED (compress=false).  The Python bridge asked
//    for PNG and paid for UE-side PNG compression plus PIL decode — that was the single
//    biggest contributor to the 2.4 Hz stereo rate.  Raw buffers are 3 bytes/px in R,G,B
//    order (Unreal/Plugins/AirSim/Source/RenderRequest.cpp:116-120), converted to mono8 here.
//
//  * Depth is DepthPlanar (pixels_as_float), masked below depth_min_range (own-prop
//    rejection, as cuvslam_sim_bridge.py:219 does) and above depth_max_range (D435i
//    out-of-range semantics), and published as 16UC1 millimetres by default (field parity
//    with the realsense splitter) or 32FC1 metres.
//
//  * Static TFs, intrinsics (FOV 87 deg), baseline (50 mm) and the NED->FLU->optical IMU
//    rotation reproduce cuvslam_sim_bridge.py exactly, so cuVSLAM/nvblox see byte-identical
//    geometry when the Python sensor loops are switched off (NB_SENSORS=0).
//
// Topics (topic_prefix defaults to /sim, matching run_cuvslam_sim.sh / run_fleet_coord.sh):
//   <prefix>/ir_left/image        sensor_msgs/Image      mono8
//   <prefix>/ir_left/camera_info  sensor_msgs/CameraInfo
//   <prefix>/ir_right/image       sensor_msgs/Image      mono8
//   <prefix>/ir_right/camera_info sensor_msgs/CameraInfo
//   <prefix>/imu                  sensor_msgs/Imu
//   <prefix>/depth/image          sensor_msgs/Image      16UC1 (mm) | 32FC1 (m)
//   <prefix>/depth/camera_info    sensor_msgs/CameraInfo
// all on SensorDataQoS (BEST_EFFORT / KEEP_LAST), which is what realsense-ros publishes and
// what cuVSLAM (image_qos/imu_qos default SENSOR_DATA) subscribes with.  nvblox defaults to
// SYSTEM_DEFAULT and must be launched with -p input_qos:=SENSOR_DATA.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

using ImageCaptureBase = msr::airlib::ImageCaptureBase;
using ImageRequest = ImageCaptureBase::ImageRequest;
using ImageResponse = ImageCaptureBase::ImageResponse;
using ImageType = ImageCaptureBase::ImageType;

namespace
{

// ---------------------------------------------------------------------------------------
// Minimal (w, x, y, z) quaternion helpers — deliberately mirrors cuvslam_sim_bridge.py's
// qmul/qconj/qrot/q_axis so the IMU rotation and the static TFs are bit-comparable.
// ---------------------------------------------------------------------------------------
struct Quat
{
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

Quat qmul(const Quat& a, const Quat& b)
{
    Quat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

Quat qconj(const Quat& q)
{
    return Quat{ q.w, -q.x, -q.y, -q.z };
}

// Rotate vector v by quaternion q (q * [0,v] * q^-1).
void qrot(const Quat& q, const double v[3], double out[3])
{
    const Quat p{ 0.0, v[0], v[1], v[2] };
    const Quat r = qmul(qmul(q, p), qconj(q));
    out[0] = r.x;
    out[1] = r.y;
    out[2] = r.z;
}

Quat q_axis(double ax, double ay, double az, double ang)
{
    const double s = std::sin(ang / 2.0);
    return Quat{ std::cos(ang / 2.0), ax * s, ay * s, az * s };
}

// Body-FLU -> camera-optical (REP-103 -> REP-105 optical): Rz(-90) * Rx(-90) = (0.5,-0.5,0.5,-0.5).
Quat optical_from_flu()
{
    return qmul(q_axis(0, 0, 1, -M_PI / 2.0), q_axis(1, 0, 0, -M_PI / 2.0));
}

// Steady-clock pacer with BOUNDED catch-up.
//
// A pacer that re-anchors its deadline to "now" after every overrun cannot hold the target
// rate under jitter: slow cycles cost time that fast cycles are then forbidden to recover,
// so the mean lands well below the setpoint (measured: 27 Hz against a 30 Hz setpoint on a
// link whose ceiling is 41 Hz).  Allowing the deadline to lag by up to max_catchup periods
// lets a fast cycle immediately follow a slow one and pulls the mean back to the setpoint,
// while the bound still prevents an unbounded burst after a long UE render hitch.
class Pacer
{
public:
    explicit Pacer(double hz, double max_catchup_periods = 2.0)
        : period_(hz > 0.0 ? std::chrono::duration<double>(1.0 / hz) : std::chrono::duration<double>(0.0))
        , max_lag_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              period_ * std::max(0.0, max_catchup_periods)))
        , next_(std::chrono::steady_clock::now())
    {
    }

    void wait()
    {
        if (period_.count() <= 0.0) {
            return;
        }
        next_ += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
        const auto now = std::chrono::steady_clock::now();
        if (next_ > now) {
            std::this_thread::sleep_until(next_);
        }
        else if (now - next_ > max_lag_) {
            next_ = now; // fell too far behind (render hitch) — resynchronise, don't burst
        }
        // otherwise: within the catch-up budget, run the next cycle immediately
    }

private:
    std::chrono::duration<double> period_;
    std::chrono::steady_clock::duration max_lag_;
    std::chrono::steady_clock::time_point next_;
};

} // namespace

class AirsimRealsenseNode : public rclcpp::Node
{
public:
    AirsimRealsenseNode()
        : Node("airsim_realsense_node")
    {
        vehicle_name_ = declare_parameter<std::string>("vehicle_name", "ghost");
        host_ip_ = declare_parameter<std::string>("host_ip", "localhost");
        host_port_ = declare_parameter<int>("host_port", 41451);

        stereo_hz_ = declare_parameter<double>("stereo_hz", 30.0);
        imu_hz_ = declare_parameter<double>("imu_hz", 200.0);
        depth_hz_ = declare_parameter<double>("depth_hz", 30.0);

        topic_prefix_ = declare_parameter<std::string>("topic_prefix", "/sim");
        left_topic_ns_ = declare_parameter<std::string>("left_topic_ns", "ir_left");
        right_topic_ns_ = declare_parameter<std::string>("right_topic_ns", "ir_right");
        depth_topic_ns_ = declare_parameter<std::string>("depth_topic_ns", "depth");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "imu");

        depth_encoding_ = declare_parameter<std::string>("depth_encoding", "16UC1");
        if (depth_encoding_ != "16UC1" && depth_encoding_ != "32FC1") {
            RCLCPP_WARN(get_logger(), "unknown depth_encoding '%s', falling back to 16UC1",
                        depth_encoding_.c_str());
            depth_encoding_ = "16UC1";
        }

        left_camera_name_ = declare_parameter<std::string>("left_camera_name", "front_left");
        right_camera_name_ = declare_parameter<std::string>("right_camera_name", "front_right");
        depth_camera_name_ = declare_parameter<std::string>("depth_camera_name", "front_center");
        imu_name_ = declare_parameter<std::string>("imu_name", "imu");

        camera_link_frame_id_ = declare_parameter<std::string>("camera_link_frame_id", "camera0_link");
        infra1_frame_id_ = declare_parameter<std::string>("infra1_frame_id", "camera0_infra1_optical_frame");
        infra2_frame_id_ = declare_parameter<std::string>("infra2_frame_id", "camera0_infra2_optical_frame");
        depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", "camera0_depth_optical_frame");
        imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "camera0_gyro_optical_frame");

        fov_degrees_ = declare_parameter<double>("fov_degrees", 87.0);
        baseline_ = declare_parameter<double>("baseline", 0.05);
        camera_pitch_degrees_ = declare_parameter<double>("camera_pitch_degrees", 20.0);

        depth_min_range_ = declare_parameter<double>("depth_min_range", 0.35);
        depth_max_range_ = declare_parameter<double>("depth_max_range", 20.0);

        publish_static_tf_ = declare_parameter<bool>("publish_static_tf", true);
        use_sim_stamp_ = declare_parameter<bool>("use_sim_stamp", true);
        imu_dedupe_ = declare_parameter<bool>("imu_dedupe", true);
        rgb_input_ = declare_parameter<bool>("rgb_input", true);

        enable_stereo_ = declare_parameter<bool>("enable_stereo", true);
        enable_depth_ = declare_parameter<bool>("enable_depth", true);
        enable_imu_ = declare_parameter<bool>("enable_imu", true);

        max_catchup_periods_ = declare_parameter<double>("max_catchup_periods", 2.0);
        stats_period_s_ = declare_parameter<double>("stats_period_s", 10.0);
        connect_timeout_s_ = declare_parameter<double>("connect_timeout_s", 60.0);

        // Q_FLU_TO_OPT = conj(Q_CAM_PITCH * Q_OPTICAL) — the inverse of the full body->optical
        // rotation, applied to body-FLU vectors to express them in the optical frame.
        q_optical_ = optical_from_flu();
        const Quat q_pitch = q_axis(0, 1, 0, camera_pitch_degrees_ * M_PI / 180.0);
        q_flu_to_opt_ = qconj(qmul(q_pitch, q_optical_));

        auto sensor_qos = rclcpp::SensorDataQoS();

        const std::string p = topic_prefix_;
        left_pub_ = create_publisher<sensor_msgs::msg::Image>(p + "/" + left_topic_ns_ + "/image", sensor_qos);
        left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(p + "/" + left_topic_ns_ + "/camera_info", sensor_qos);
        right_pub_ = create_publisher<sensor_msgs::msg::Image>(p + "/" + right_topic_ns_ + "/image", sensor_qos);
        right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(p + "/" + right_topic_ns_ + "/camera_info", sensor_qos);
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>(p + "/" + depth_topic_ns_ + "/image", sensor_qos);
        depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(p + "/" + depth_topic_ns_ + "/camera_info", sensor_qos);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(p + "/" + imu_topic_, sensor_qos);

        if (publish_static_tf_) {
            publish_static_transforms();
        }

        RCLCPP_INFO(get_logger(),
                    "airsim_realsense_node vehicle=%s target rates: stereo %.1f Hz, depth %.1f Hz (%s), imu %.1f Hz",
                    vehicle_name_.c_str(), stereo_hz_, depth_hz_, depth_encoding_.c_str(), imu_hz_);
    }

    ~AirsimRealsenseNode() override
    {
        stop();
    }

    void start()
    {
        running_.store(true);
        t_start_ = std::chrono::steady_clock::now();
        t_last_stats_ = t_start_;
        if (enable_stereo_) {
            threads_.emplace_back(&AirsimRealsenseNode::stereo_loop, this);
        }
        if (enable_depth_) {
            threads_.emplace_back(&AirsimRealsenseNode::depth_loop, this);
        }
        if (enable_imu_) {
            threads_.emplace_back(&AirsimRealsenseNode::imu_loop, this);
        }
        if (stats_period_s_ > 0.0) {
            stats_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(stats_period_s_)),
                std::bind(&AirsimRealsenseNode::log_stats, this));
        }
    }

    void stop()
    {
        if (!running_.exchange(false)) {
            return;
        }
        for (auto& t : threads_) {
            if (t.joinable()) {
                t.join();
            }
        }
        threads_.clear();
    }

private:
    // -----------------------------------------------------------------------------------
    // setup
    // -----------------------------------------------------------------------------------
    void publish_static_transforms()
    {
        static_bc_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        const auto stamp = now();
        std::vector<geometry_msgs::msg::TransformStamped> tfs;
        // Baseline is split symmetrically about camera0_link, matching cuvslam_sim_bridge.py:131-138
        // and settings-fleet-*drone.json (front_left Y=-0.025, front_right Y=+0.025 in NED, i.e.
        // +0.025 / -0.025 in FLU).
        tfs.push_back(make_tf(stamp, camera_link_frame_id_, infra1_frame_id_, 0.0, baseline_ / 2.0, 0.0));
        tfs.push_back(make_tf(stamp, camera_link_frame_id_, infra2_frame_id_, 0.0, -baseline_ / 2.0, 0.0));
        tfs.push_back(make_tf(stamp, camera_link_frame_id_, depth_frame_id_, 0.0, 0.0, 0.0));
        tfs.push_back(make_tf(stamp, camera_link_frame_id_, imu_frame_id_, 0.0, 0.0, 0.0));
        static_bc_->sendTransform(tfs);
    }

    geometry_msgs::msg::TransformStamped make_tf(const rclcpp::Time& stamp,
                                                 const std::string& parent, const std::string& child,
                                                 double tx, double ty, double tz) const
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = tx;
        t.transform.translation.y = ty;
        t.transform.translation.z = tz;
        t.transform.rotation.w = q_optical_.w;
        t.transform.rotation.x = q_optical_.x;
        t.transform.rotation.y = q_optical_.y;
        t.transform.rotation.z = q_optical_.z;
        return t;
    }

    sensor_msgs::msg::CameraInfo make_camera_info(int width, int height,
                                                  const std::string& frame_id, double tx) const
    {
        sensor_msgs::msg::CameraInfo ci;
        const double fx = (width / 2.0) / std::tan(fov_degrees_ * M_PI / 180.0 / 2.0);
        ci.header.frame_id = frame_id;
        ci.width = static_cast<uint32_t>(width);
        ci.height = static_cast<uint32_t>(height);
        ci.distortion_model = "plumb_bob";
        ci.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };
        ci.k = { fx, 0.0, width / 2.0, 0.0, fx, height / 2.0, 0.0, 0.0, 1.0 };
        ci.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
        ci.p = { fx, 0.0, width / 2.0, -tx * fx, 0.0, fx, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0 };
        return ci;
    }

    // AirSim TTimePoint is nanoseconds since the Unix epoch off the sim ClockBase
    // (ScalableClock at scale 1 == wall clock), so it drops straight into an rclcpp::Time.
    rclcpp::Time stamp_from(uint64_t sim_ns)
    {
        if (use_sim_stamp_ && sim_ns > 0) {
            return rclcpp::Time(static_cast<int64_t>(sim_ns), RCL_ROS_TIME);
        }
        return now();
    }

    std::unique_ptr<msr::airlib::MultirotorRpcLibClient> connect(const char* who)
    {
        auto client = std::make_unique<msr::airlib::MultirotorRpcLibClient>(host_ip_, host_port_);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                  std::chrono::duration<double>(connect_timeout_s_));
        while (running_.load() && rclcpp::ok()) {
            try {
                client->confirmConnection();
                RCLCPP_INFO(get_logger(), "[%s] connected to AirSim at %s:%d (vehicle %s)",
                            who, host_ip_.c_str(), host_port_, vehicle_name_.c_str());
                return client;
            }
            catch (const std::exception& e) {
                if (std::chrono::steady_clock::now() > deadline) {
                    RCLCPP_ERROR(get_logger(), "[%s] could not connect to AirSim: %s", who, e.what());
                    return nullptr;
                }
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "[%s] waiting for AirSim at %s:%d (%s)",
                                     who, host_ip_.c_str(), host_port_, e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
        return nullptr;
    }

    // -----------------------------------------------------------------------------------
    // stereo — one simGetImages batch of the two Scene cameras on a dedicated connection
    // -----------------------------------------------------------------------------------
    void stereo_loop()
    {
        auto client = connect("stereo");
        if (!client) {
            return;
        }
        const std::vector<ImageRequest> reqs = {
            ImageRequest(left_camera_name_, ImageType::Scene, false, false),
            ImageRequest(right_camera_name_, ImageType::Scene, false, false)
        };

        Pacer pacer(stereo_hz_, max_catchup_periods_);
        sensor_msgs::msg::Image left_msg, right_msg;
        left_msg.header.frame_id = infra1_frame_id_;
        right_msg.header.frame_id = infra2_frame_id_;
        left_msg.encoding = right_msg.encoding = "mono8";
        left_msg.is_bigendian = right_msg.is_bigendian = 0;

        bool info_ready = false;
        sensor_msgs::msg::CameraInfo left_info, right_info;

        while (running_.load() && rclcpp::ok()) {
            std::vector<ImageResponse> responses;
            try {
                responses = client->simGetImages(reqs, vehicle_name_);
            }
            catch (const std::exception& e) {
                stereo_errors_.fetch_add(1);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[stereo] RPC error: %s", e.what());
                pacer.wait();
                continue;
            }

            if (responses.size() < 2 || !valid_uint8(responses[0]) || !valid_uint8(responses[1])) {
                stereo_errors_.fetch_add(1);
                pacer.wait();
                continue;
            }

            // Q9 verifier fix: batch stamps are NOT guaranteed identical — stamp BOTH images
            // and BOTH camera_infos from responses[0] so cuVSLAM sees a matched pair.
            const rclcpp::Time stamp = stamp_from(responses[0].time_stamp);

            if (!info_ready || left_info.width != static_cast<uint32_t>(responses[0].width) ||
                left_info.height != static_cast<uint32_t>(responses[0].height)) {
                left_info = make_camera_info(responses[0].width, responses[0].height, infra1_frame_id_, 0.0);
                right_info = make_camera_info(responses[1].width, responses[1].height, infra2_frame_id_, baseline_);
                info_ready = true;
            }

            fill_mono8(responses[0], left_msg);
            fill_mono8(responses[1], right_msg);
            left_msg.header.stamp = stamp;
            right_msg.header.stamp = stamp;
            left_info.header.stamp = stamp;
            right_info.header.stamp = stamp;

            left_pub_->publish(left_msg);
            right_pub_->publish(right_msg);
            left_info_pub_->publish(left_info);
            right_info_pub_->publish(right_info);
            stereo_pairs_.fetch_add(1);

            pacer.wait();
        }
    }

    static bool valid_uint8(const ImageResponse& r)
    {
        return r.width > 0 && r.height > 0 &&
               r.image_data_uint8.size() >= static_cast<size_t>(r.width) * r.height * 3u;
    }

    // Uncompressed AirSim Scene buffers are 3 bytes/px in R,G,B order
    // (Unreal/Plugins/AirSim/Source/RenderRequest.cpp:116-120).  Field infra1/infra2 are mono8,
    // so collapse to ITU-R BT.601 luma here rather than shipping colour cuVSLAM would discard.
    void fill_mono8(const ImageResponse& r, sensor_msgs::msg::Image& msg) const
    {
        const size_t w = static_cast<size_t>(r.width);
        const size_t h = static_cast<size_t>(r.height);
        msg.width = static_cast<uint32_t>(w);
        msg.height = static_cast<uint32_t>(h);
        msg.step = static_cast<uint32_t>(w);
        msg.data.resize(w * h);

        const uint8_t* src = r.image_data_uint8.data();
        uint8_t* dst = msg.data.data();
        const size_t n = w * h;
        if (rgb_input_) {
            for (size_t i = 0; i < n; ++i, src += 3) {
                dst[i] = static_cast<uint8_t>((src[0] * 4899u + src[1] * 9617u + src[2] * 1868u) >> 14);
            }
        }
        else {
            for (size_t i = 0; i < n; ++i, src += 3) {
                dst[i] = static_cast<uint8_t>((src[2] * 4899u + src[1] * 9617u + src[0] * 1868u) >> 14);
            }
        }
    }

    // -----------------------------------------------------------------------------------
    // depth — DepthPlanar float on its own connection
    // -----------------------------------------------------------------------------------
    void depth_loop()
    {
        auto client = connect("depth");
        if (!client) {
            return;
        }
        const std::vector<ImageRequest> reqs = {
            ImageRequest(depth_camera_name_, ImageType::DepthPlanar, true, false)
        };

        Pacer pacer(depth_hz_, max_catchup_periods_);
        sensor_msgs::msg::Image msg;
        msg.header.frame_id = depth_frame_id_;
        msg.is_bigendian = 0;
        const bool as_u16 = (depth_encoding_ == "16UC1");
        msg.encoding = as_u16 ? "16UC1" : "32FC1";

        bool info_ready = false;
        sensor_msgs::msg::CameraInfo info;

        while (running_.load() && rclcpp::ok()) {
            std::vector<ImageResponse> responses;
            try {
                responses = client->simGetImages(reqs, vehicle_name_);
            }
            catch (const std::exception& e) {
                depth_errors_.fetch_add(1);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[depth] RPC error: %s", e.what());
                pacer.wait();
                continue;
            }

            if (responses.empty() || responses[0].width == 0 || responses[0].height == 0 ||
                responses[0].image_data_float.size() <
                    static_cast<size_t>(responses[0].width) * responses[0].height) {
                depth_errors_.fetch_add(1);
                pacer.wait();
                continue;
            }

            const ImageResponse& r = responses[0];
            const rclcpp::Time stamp = stamp_from(r.time_stamp);
            const size_t n = static_cast<size_t>(r.width) * r.height;

            msg.width = static_cast<uint32_t>(r.width);
            msg.height = static_cast<uint32_t>(r.height);
            if (as_u16) {
                msg.step = static_cast<uint32_t>(r.width * sizeof(uint16_t));
                msg.data.resize(n * sizeof(uint16_t));
                uint16_t* dst = reinterpret_cast<uint16_t*>(msg.data.data());
                for (size_t i = 0; i < n; ++i) {
                    dst[i] = to_mm(r.image_data_float[i]);
                }
            }
            else {
                msg.step = static_cast<uint32_t>(r.width * sizeof(float));
                msg.data.resize(n * sizeof(float));
                float* dst = reinterpret_cast<float*>(msg.data.data());
                for (size_t i = 0; i < n; ++i) {
                    dst[i] = to_metres(r.image_data_float[i]);
                }
            }
            msg.header.stamp = stamp;

            if (!info_ready || info.width != msg.width || info.height != msg.height) {
                info = make_camera_info(r.width, r.height, depth_frame_id_, 0.0);
                info_ready = true;
            }
            info.header.stamp = stamp;

            depth_pub_->publish(msg);
            depth_info_pub_->publish(info);
            depth_frames_.fetch_add(1);

            pacer.wait();
        }
    }

    // Own-prop rejection below depth_min_range and out-of-range above depth_max_range both map
    // to the invalid-depth sentinel (0), matching realsense splitter / D435i semantics and
    // cuvslam_sim_bridge.py:219.
    inline uint16_t to_mm(float d) const
    {
        if (!std::isfinite(d) || d < depth_min_range_ || (depth_max_range_ > 0.0 && d > depth_max_range_)) {
            return 0;
        }
        const double mm = static_cast<double>(d) * 1000.0;
        return static_cast<uint16_t>(std::min(mm, 65535.0));
    }

    inline float to_metres(float d) const
    {
        if (!std::isfinite(d) || d < depth_min_range_ || (depth_max_range_ > 0.0 && d > depth_max_range_)) {
            return 0.0f;
        }
        return d;
    }

    // -----------------------------------------------------------------------------------
    // IMU — getImuData on its own connection.  This is the stream the SimModeBase worker-pool
    // patch exists to protect: it must keep 200 Hz while 4 stereo + 4 depth renders are in
    // flight.  AirSim physics ticks at 3 ms (SimModeWorldBase.h:75 = 333 Hz), so polling at
    // 200 Hz yields unique samples; identical stamps are dropped rather than republished,
    // because a duplicate IMU sample is worse for cuVSLAM than a missing one.
    // -----------------------------------------------------------------------------------
    void imu_loop()
    {
        auto client = connect("imu");
        if (!client) {
            return;
        }
        Pacer pacer(imu_hz_, max_catchup_periods_);
        sensor_msgs::msg::Imu msg;
        msg.header.frame_id = imu_frame_id_;
        msg.orientation_covariance[0] = -1.0; // orientation not supplied, per REP-145

        uint64_t last_stamp = 0;
        while (running_.load() && rclcpp::ok()) {
            msr::airlib::ImuBase::Output data;
            try {
                data = client->getImuData(imu_name_, vehicle_name_);
            }
            catch (const std::exception& e) {
                imu_errors_.fetch_add(1);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "[imu] RPC error: %s", e.what());
                pacer.wait();
                continue;
            }

            if (imu_dedupe_ && data.time_stamp != 0 && data.time_stamp == last_stamp) {
                imu_dupes_.fetch_add(1);
                pacer.wait();
                continue;
            }
            last_stamp = data.time_stamp;

            // AirSim reports in body NED.  body NED -> body FLU (x, -y, -z), then FLU -> optical.
            const double g_flu[3] = { data.angular_velocity.x(), -data.angular_velocity.y(),
                                      -data.angular_velocity.z() };
            const double a_flu[3] = { data.linear_acceleration.x(), -data.linear_acceleration.y(),
                                      -data.linear_acceleration.z() };
            double g_opt[3], a_opt[3];
            qrot(q_flu_to_opt_, g_flu, g_opt);
            qrot(q_flu_to_opt_, a_flu, a_opt);

            msg.header.stamp = stamp_from(data.time_stamp);
            msg.angular_velocity.x = g_opt[0];
            msg.angular_velocity.y = g_opt[1];
            msg.angular_velocity.z = g_opt[2];
            msg.linear_acceleration.x = a_opt[0];
            msg.linear_acceleration.y = a_opt[1];
            msg.linear_acceleration.z = a_opt[2];

            imu_pub_->publish(msg);
            imu_samples_.fetch_add(1);

            pacer.wait();
        }
    }

    // -----------------------------------------------------------------------------------
    // per-stream counters — Q9 moves these out of the (now gutted) Python bridge
    // -----------------------------------------------------------------------------------
    void log_stats()
    {
        const auto tnow = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(tnow - t_last_stats_).count();
        const double total = std::chrono::duration<double>(tnow - t_start_).count();
        t_last_stats_ = tnow;
        if (dt <= 0.0) {
            return;
        }

        const uint64_t s = stereo_pairs_.load(), i = imu_samples_.load(), d = depth_frames_.load();
        const uint64_t ds = s - last_stereo_, di = i - last_imu_, dd = d - last_depth_;
        last_stereo_ = s;
        last_imu_ = i;
        last_depth_ = d;

        RCLCPP_INFO(get_logger(),
                    "[%s] %.0fs | stereo %.2f Hz (%lu pairs) | imu %.1f Hz (%lu) | depth %.2f Hz (%lu) "
                    "| err s/d/i %lu/%lu/%lu | imu dupes %lu",
                    vehicle_name_.c_str(), total,
                    ds / dt, static_cast<unsigned long>(s),
                    di / dt, static_cast<unsigned long>(i),
                    dd / dt, static_cast<unsigned long>(d),
                    static_cast<unsigned long>(stereo_errors_.load()),
                    static_cast<unsigned long>(depth_errors_.load()),
                    static_cast<unsigned long>(imu_errors_.load()),
                    static_cast<unsigned long>(imu_dupes_.load()));
    }

    // params
    std::string vehicle_name_, host_ip_, topic_prefix_, depth_encoding_;
    std::string left_topic_ns_, right_topic_ns_, depth_topic_ns_, imu_topic_;
    std::string left_camera_name_, right_camera_name_, depth_camera_name_, imu_name_;
    std::string camera_link_frame_id_, infra1_frame_id_, infra2_frame_id_, depth_frame_id_, imu_frame_id_;
    int host_port_{ 41451 };
    double stereo_hz_{ 30.0 }, imu_hz_{ 200.0 }, depth_hz_{ 30.0 };
    double fov_degrees_{ 87.0 }, baseline_{ 0.05 }, camera_pitch_degrees_{ 20.0 };
    double depth_min_range_{ 0.35 }, depth_max_range_{ 20.0 };
    double stats_period_s_{ 10.0 }, connect_timeout_s_{ 60.0 }, max_catchup_periods_{ 2.0 };
    bool publish_static_tf_{ true }, use_sim_stamp_{ true }, imu_dedupe_{ true }, rgb_input_{ true };
    bool enable_stereo_{ true }, enable_depth_{ true }, enable_imu_{ true };

    Quat q_optical_, q_flu_to_opt_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_, depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_, right_info_pub_, depth_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_bc_;
    rclcpp::TimerBase::SharedPtr stats_timer_;

    std::atomic<bool> running_{ false };
    std::vector<std::thread> threads_;

    std::atomic<uint64_t> stereo_pairs_{ 0 }, imu_samples_{ 0 }, depth_frames_{ 0 };
    std::atomic<uint64_t> stereo_errors_{ 0 }, depth_errors_{ 0 }, imu_errors_{ 0 }, imu_dupes_{ 0 };
    uint64_t last_stereo_{ 0 }, last_imu_{ 0 }, last_depth_{ 0 };
    std::chrono::steady_clock::time_point t_start_, t_last_stats_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AirsimRealsenseNode>();
    node->start();
    rclcpp::spin(node);
    node->stop();
    rclcpp::shutdown();
    return 0;
}
