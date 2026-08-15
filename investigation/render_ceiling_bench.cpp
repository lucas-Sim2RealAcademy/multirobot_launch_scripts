// rpcbench.cpp — standalone AirSim RPC render-throughput benchmark.
//
// Mirrors airsim_realsense_node.cpp's topology exactly: ONE RPC client (== one TCP
// connection == one rpclib server worker thread) per requesting stream, each issuing
// blocking simGetImages() in a tight loop.  Counts DELIVERED RENDERS (images), not calls.
//
// Usage:
//   rpcbench --clients N --dur SEC [--mode scene|depth|stereo|stereo+depth]
//            [--compress 0|1] [--float 0|1] [--hz 0]  [--vehicles a,b,c,d]
//
// --hz 0 == unpaced (measure ceiling).  --hz X == paced (measure achieved-vs-target).

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using ImageCaptureBase = msr::airlib::ImageCaptureBase;
using ImageRequest = ImageCaptureBase::ImageRequest;
using ImageResponse = ImageCaptureBase::ImageResponse;
using ImageType = ImageCaptureBase::ImageType;
using clk = std::chrono::steady_clock;

struct Stats {
    std::atomic<long> calls{0};
    std::atomic<long> renders{0};
    std::atomic<long> errors{0};
    std::atomic<long> bytes{0};
    std::vector<double> lat_ms;   // guarded by mtx
    std::mutex mtx;
};

static std::vector<std::string> split(const std::string& s, char d) {
    std::vector<std::string> out; std::string cur;
    for (char c : s) { if (c == d) { if (!cur.empty()) out.push_back(cur); cur.clear(); } else cur += c; }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

int main(int argc, char** argv)
{
    int nclients = 1, dur = 15, compress = 0, pixels_as_float = -1;
    double hz = 0.0;
    std::string mode = "stereo";
    std::string vehstr = "ghost,delta,buckshee,thunderstrike";
    std::string host = "127.0.0.1";
    int port = 41451;

    for (int i = 1; i < argc - 1; ++i) {
        std::string a = argv[i];
        if (a == "--clients") nclients = atoi(argv[++i]);
        else if (a == "--dur") dur = atoi(argv[++i]);
        else if (a == "--mode") mode = argv[++i];
        else if (a == "--compress") compress = atoi(argv[++i]);
        else if (a == "--float") pixels_as_float = atoi(argv[++i]);
        else if (a == "--hz") hz = atof(argv[++i]);
        else if (a == "--vehicles") vehstr = argv[++i];
        else if (a == "--host") host = argv[++i];
        else if (a == "--port") port = atoi(argv[++i]);
    }

    std::vector<std::string> vehicles = split(vehstr, ',');

    // Build the request list for this mode, matching airsim_realsense_node.cpp.
    auto make_reqs = [&](void) -> std::vector<ImageRequest> {
        if (mode == "scene")
            return { ImageRequest("front_left", ImageType::Scene, compress ? false : (pixels_as_float == 1), compress != 0) };
        if (mode == "depth")
            return { ImageRequest("front_center", ImageType::DepthPlanar,
                                  pixels_as_float == 0 ? false : true, compress != 0) };
        if (mode == "stereo")
            return { ImageRequest("front_left",  ImageType::Scene, false, compress != 0),
                     ImageRequest("front_right", ImageType::Scene, false, compress != 0) };
        if (mode == "stereo+depth")
            return { ImageRequest("front_left",  ImageType::Scene, false, compress != 0),
                     ImageRequest("front_right", ImageType::Scene, false, compress != 0),
                     ImageRequest("front_center", ImageType::DepthPlanar, true, compress != 0) };
        fprintf(stderr, "bad mode\n"); exit(2);
    };

    Stats st;
    std::atomic<bool> run{true};
    std::atomic<int> ready{0};
    std::vector<std::thread> threads;
    std::vector<long> per_client_renders(nclients, 0);
    std::vector<int>  per_client_w(nclients, 0), per_client_h(nclients, 0);

    for (int c = 0; c < nclients; ++c) {
        threads.emplace_back([&, c]() {
            const std::string veh = vehicles[c % vehicles.size()];
            msr::airlib::MultirotorRpcLibClient client(host, port, 60);
            try { client.confirmConnection(); }
            catch (...) { st.errors.fetch_add(1); ready.fetch_add(1); return; }
            const auto reqs = make_reqs();

            std::vector<double> local_lat;
            local_lat.reserve(4096);
            long local_renders = 0;
            ready.fetch_add(1);
            while (ready.load() < nclients) std::this_thread::sleep_for(std::chrono::milliseconds(2));

            auto next = clk::now();
            const auto period = std::chrono::duration<double>(hz > 0 ? 1.0 / hz : 0.0);

            while (run.load()) {
                auto t0 = clk::now();
                std::vector<ImageResponse> resp;
                try { resp = client.simGetImages(reqs, veh); }
                catch (const std::exception&) { st.errors.fetch_add(1); continue; }
                auto t1 = clk::now();
                double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
                local_lat.push_back(ms);

                long good = 0, b = 0;
                for (const auto& r : resp) {
                    if (r.width > 0 && r.height > 0 &&
                        (r.image_data_uint8.size() > 0 || r.image_data_float.size() > 0)) {
                        ++good;
                        b += (long)r.image_data_uint8.size() + (long)r.image_data_float.size() * 4;
                        per_client_w[c] = r.width; per_client_h[c] = r.height;
                    }
                }
                st.calls.fetch_add(1);
                st.renders.fetch_add(good);
                local_renders += good;
                st.bytes.fetch_add(b);

                if (hz > 0) {
                    next += std::chrono::duration_cast<clk::duration>(period);
                    auto now = clk::now();
                    if (next > now) std::this_thread::sleep_for(next - now);
                    else next = now;   // no unbounded catch-up
                }
            }
            per_client_renders[c] = local_renders;
            std::lock_guard<std::mutex> lk(st.mtx);
            st.lat_ms.insert(st.lat_ms.end(), local_lat.begin(), local_lat.end());
        });
    }

    while (ready.load() < nclients) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto tstart = clk::now();
    std::this_thread::sleep_for(std::chrono::seconds(dur));
    run.store(false);
    for (auto& t : threads) t.join();
    double elapsed = std::chrono::duration<double>(clk::now() - tstart).count();

    auto& L = st.lat_ms;
    std::sort(L.begin(), L.end());
    auto pct = [&](double p) { return L.empty() ? 0.0 : L[std::min(L.size() - 1, (size_t)(p * L.size()))]; };

    printf("RESULT mode=%s clients=%d compress=%d hz=%.1f res=%dx%d elapsed=%.2f "
           "calls=%ld renders=%ld errors=%ld renders_per_s=%.2f calls_per_s=%.2f "
           "MBps=%.1f lat_p50=%.2f lat_p90=%.2f lat_p99=%.2f\n",
           mode.c_str(), nclients, compress, hz, per_client_w[0], per_client_h[0], elapsed,
           st.calls.load(), st.renders.load(), st.errors.load(),
           st.renders.load() / elapsed, st.calls.load() / elapsed,
           st.bytes.load() / elapsed / 1e6, pct(0.50), pct(0.90), pct(0.99));
    for (int c = 0; c < nclients; ++c)
        printf("  client %d renders/s %.2f\n", c, per_client_renders[c] / elapsed);
    return 0;
}
