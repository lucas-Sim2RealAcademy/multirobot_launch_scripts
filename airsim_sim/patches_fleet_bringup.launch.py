#!/usr/bin/env python3
"""fleet_bringup: one launch per vehicle for the multi-drone field stack
(spec block 9).

    ros2 launch multi_drone_nvblox fleet_bringup.launch.py \
        drone_id:=3 alignment_yaml:=/path/swarm_alignment.yaml

Namespace architecture (contradiction resolution, reported in block 9):
the spec asks for every node under /d<i>, but realsense_example.launch.py
and its component containers use absolute namespaces internally, so a
PushRosNamespace wrapper produces a half-namespaced split-brain stack
(camera publishing under /d<i> while cuVSLAM/nvblox listen at root --
observed live on ghost, 2026-08-03). Resolution: vehicles are already
isolated from each other by ROS_DOMAIN_ID (= vehicle id), so the LEGACY
single-vehicle stack (camera, cuVSLAM, nvblox, vio_bridge+XRCE agent,
shared mapper, FIS, guard, planner) runs at root exactly as in the proven
single-drone launch scripts, and only the NEW coordination nodes -- whose
topics cross the zenoh bridge or address peer namespaces -- run under
/d<i>. Boundary edges between the two worlds are explicit remaps/params
below. Nothing outside /d<i>/... is bridged (gen_zenoh_config.py
whitelist), so root topics can never collide across vehicles.

Stages (timers; every stage tolerates the previous still warming up
because every node degrades gracefully):

  t=0   root: realsense_example (camera, cuVSLAM, nvblox, pose source
        cuvslam|ekf2) + px4_offboard vio_bridge.launch.py (XRCE agent +
        vio_bridge)
  t=8   root: shared mapper (second nvblox, 4-camera)
  t=14  /d<i>: alignment_manager, lora_bridge, coordination_node,
        link_fault_injector (optional)
  t=18  root: FIS + reactive depth guard
  t=22  /d<i>: keyframe_exchange + peer_map_integrator
  t=26  root: exploration planner

The vehicle's ROS_DOMAIN_ID comes from the environment (must equal PX4
UXRCE_DDS_DOM_ID; verified by preflight_check.py, never forced here).

pose_source: 'ekf2' is the paper configuration (invariant 8); 'cuvslam' is
for bench work where the disarmed landed-state clamp pins EKF2 z. Default is
cuvslam until the armed EKF2 hover validation passes; flip the default after.

connectivity_mode: full | lora_only | wifi_only (lora_only skips keyframe
exchange; wifi_only skips the lora bridge). Vehicle loss/partition testing
uses link_fault_injector instead.
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from mdn_core.geofence_logic import team_forward_box


def _stage(t, actions):
    return TimerAction(period=float(t), actions=actions)


# tf2_ros subscribes/publishes RELATIVE 'tf'/'tf_static': under /d<i> that
# becomes /d<i>/tf, invisible to the root-level legacy stack (found live on
# the bench 2026-08-03: keyframe_exchange could never resolve camera TF).
# Every namespaced node that touches TF gets remapped onto the global tree.
def _setup(context, *args, **kwargs):
    drone_id = int(LaunchConfiguration('drone_id').perform(context))
    ns = '%s%d' % (LaunchConfiguration('namespace_prefix').perform(context),
                   drone_id)
    alignment_yaml = LaunchConfiguration('alignment_yaml').perform(context)
    pose_source = LaunchConfiguration('pose_source').perform(context)
    conn = LaunchConfiguration('connectivity_mode').perform(context)
    use_fault_injector = (LaunchConfiguration(
        'use_fault_injector').perform(context).lower() == 'true')
    trust_prior = (LaunchConfiguration(
        'trust_prior_alignment').perform(context).lower() == 'true')
    flight_height = LaunchConfiguration('flight_height').perform(context)
    gf_fwd = float(LaunchConfiguration('geofence_forward_m').perform(context))
    gf_wid = float(LaunchConfiguration('geofence_width_m').perform(context))
    gf_back = float(LaunchConfiguration('geofence_behind_m').perform(context))
    debug_arm = LaunchConfiguration('debug_skip_arm_check').perform(context)

    nvblox_bringup = get_package_share_directory('nvblox_examples_bringup')
    mdn = get_package_share_directory('multi_drone_nvblox')
    ae = get_package_share_directory('active_exploration')
    px4o = get_package_share_directory('px4_offboard')

    common = {'vehicle_id': drone_id}

    # ---- t=0 (root): estimation + self mapper + XRCE agent + vio bridge ----
    est = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nvblox_bringup, 'launch', 'realsense_example.launch.py')),
        launch_arguments={
            'run_rviz': 'False',
            'pose_source': pose_source,
        }.items())
    vio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            px4o, 'launch', 'vio_bridge.launch.py')))

    # ---- t=8 (root): shared mapper ----
    shared = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mdn, 'launch', 'shared_mapper.launch.py')),
        launch_arguments={
            'own_depth_topic': '/camera0/depth/image_rect_raw',
            'own_camera_info_topic': '/camera0/depth/camera_info',
            'global_frame': 'odom',
        }.items())

    # ---- t=14 (/d<i>): all coordination nodes (single definition,
    # shared with tab 7 of launch_fleet_tmux.sh) ----
    coord = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mdn, 'launch', 'coordination_stage.launch.py')),
        launch_arguments={
            'drone_id': str(drone_id),
            'alignment_yaml': alignment_yaml,
            'trust_prior_alignment': 'true' if trust_prior else 'false',
            'connectivity_mode': conn,
            'use_fault_injector': 'true' if use_fault_injector else 'false',
        }.items())

    # ---- t=18 (root): FIS + guard ----
    # BUG-LIST-VERIFIED.md #8 (fis-bbox): FIS gets the SAME team box the
    # planner stage computes below. Without this it queries nvblox over the
    # hardcoded +/-10 m AABB of frontier_info_structure_node.cpp:18-23, pinned
    # to this vehicle's own launch origin, while the planner geofences a
    # geofence_forward_m-deep team box -- so most of the mission area is never
    # even scanned for frontiers, and roughly half of what FIS does produce
    # sits behind the launch line where the planner rejects it.
    bx0, by0, bx1, by1, box_warn = team_forward_box(
        alignment_yaml, drone_id, gf_fwd, gf_wid, gf_back)
    fis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ae, 'launch', 'fis.launch.py')),
        launch_arguments={
            'flight_height': flight_height,
            'bbox_min_x': '%.3f' % bx0,
            'bbox_min_y': '%.3f' % by0,
            'bbox_max_x': '%.3f' % bx1,
            'bbox_max_y': '%.3f' % by1,
        }.items())
    guard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ae, 'launch', 'reactive_guard.launch.py')))

    # ---- t=26 (root): planner with the team geofence (single definition,
    # shared with tab 5 of launch_fleet_tmux.sh) ----
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            mdn, 'launch', 'planner_stage.launch.py')),
        launch_arguments={
            'drone_id': str(drone_id),
            'alignment_yaml': alignment_yaml,
            'flight_height': flight_height,
            'debug_skip_arm_check': debug_arm,
            'geofence_forward_m': '%g' % gf_fwd,
            'geofence_width_m': '%g' % gf_wid,
            'geofence_behind_m': '%g' % gf_back,
        }.items())

    box_log = [LogInfo(msg='[fleet_bringup] d%d team box (own odom frame), '
                           'shared by FIS and planner: x %.2f..%.2f  '
                           'y %.2f..%.2f' % (drone_id, bx0, bx1, by0, by1))]
    if box_warn:
        box_log.append(LogInfo(msg='[fleet_bringup] WARNING: ' + box_warn))

    return box_log + [
        _stage(0.0, [est, vio]),
        _stage(8.0, [shared]),
        _stage(14.0, [coord]),
        _stage(18.0, [fis, guard]),
        _stage(26.0, [planner]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id'),
        DeclareLaunchArgument('namespace_prefix', default_value='d'),
        DeclareLaunchArgument('alignment_yaml', default_value=''),
        DeclareLaunchArgument('pose_source', default_value='cuvslam',
                              description='cuvslam (bench) | ekf2 (flight, '
                                          'invariant 8; pending validation)'),
        DeclareLaunchArgument('connectivity_mode', default_value='full',
                              choices=['full', 'lora_only', 'wifi_only']),
        DeclareLaunchArgument('use_fault_injector', default_value='false'),
        DeclareLaunchArgument('trust_prior_alignment', default_value='false',
                              description='integrate peer keyframes on the '
                                          'measured yaml prior (bench tests '
                                          'without loop closures)'),
        DeclareLaunchArgument('flight_height', default_value='1.0'),
        DeclareLaunchArgument('geofence_forward_m', default_value='35.0',
                              description='exploration depth ahead of the '
                                          'launch line (team frame)'),
        DeclareLaunchArgument('geofence_width_m', default_value='35.0',
                              description='exploration width, centred on '
                                          'the team launch spread'),
        DeclareLaunchArgument('geofence_behind_m', default_value='0.0',
                              description='tolerance behind the launch '
                                          'line; 0 = never go back past it'),
        DeclareLaunchArgument('debug_skip_arm_check', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
