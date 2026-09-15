# Prior Single-Arm Routing Experiment

## Build

The prior planner uses the extended MoveIt Task Constructor tree:

```text
~/ws_humble/src/moveit_task_constructor
```

Build the development overlay:

```bash
cd ~/ws_humble

source /opt/ros/humble/setup.bash
source ~/ws_humble/install/setup.bash

colcon --log-base ~/ws_humble/log_mtc_dev build \
  --base-paths \
    ~/ws_humble/src/moveit_task_constructor \
    ~/ws_humble/src/mtc_tutorial \
  --packages-select \
    moveit_task_constructor_msgs \
    rviz_marker_tools \
    moveit_task_constructor_core \
    moveit_task_constructor_visualization \
    mtc_tutorial \
  --allow-overriding \
    moveit_task_constructor_msgs \
    rviz_marker_tools \
    moveit_task_constructor_core \
    moveit_task_constructor_visualization \
  --build-base ~/ws_humble/build_mtc_dev \
  --install-base ~/ws_humble/install_mtc_dev \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

After building:

```bash
source ~/ws_humble/install_mtc_dev/setup.bash
```

Check the package:

```bash
ros2 pkg prefix mtc_tutorial
```

It should print:

```text
/home/tp2/ws_humble/install_mtc_dev/mtc_tutorial
```

## Start MoveIt

Start fake or real robot MoveIt first. For simulation:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_humble/install/setup.bash
source ~/ws_humble/install_mtc_dev/setup.bash

ros2 launch moveit2_tutorials isaac_demo_dualarms.launch.py \
  ros2_control_hardware_type:=mock_components
```

## Load DLO and Scene

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_humble/install/setup.bash
source ~/ws_humble/install_mtc_dev/setup.bash

ros2 launch mtc_tutorial load_dlo.launch.py
```

Load the environment scene. Use the world-coordinate environment file and
the QB-coordinate clip file:

```bash
ros2 launch mtc_tutorial load_scene.launch.py \
  scene_file:=/home/tp2/ws_humble/scene/trans/trans_env_adapt_z.scene \
  mesh_file:=/home/tp2/ws_humble/scene/trans/mesh/qb_board_plane_with_obs_2.stl \
  clip_file:=/home/tp2/ws_humble/scene/trans/target_shape_plane_7_clip_qb.scene \
  use_qb_board_coordinate:=true \
  add_clip_hats:=true
```

## Start Prior Planner

Do not start `pick_place_demo_dual.launch.py` at the same time.

```bash
ros2 launch mtc_tutorial prior_single_arm_planning.launch.py \
  start_move_group:=false \
  send_to_robot:=false
```

Use `send_to_robot:=true` when the follower trajectory server is running:

```bash
ros2 launch mtc_tutorial prior_single_arm_planning.launch.py \
  start_move_group:=false \
  send_to_robot:=true \
  follower_ip:=10.157.174.87 \
  follower_port:=12345
```

`start_move_group:=false` is required when
`isaac_demo_dualarms.launch.py` is already running.

## Test Communication

The planner request/response path can be tested without robot execution:

```bash
python3 /home/tp2/Documents/mios-wiring/python/test_prior_transition_comm.py
```

Expected topics:

```text
/prior_transition/plan_request
/prior_transition/plan_response
/prior_transition/sync_state_request
/prior_transition/sync_state_response
/prior_transition/execute_request
/prior_transition/execute_response
/prior_transition/follower_subtrajectory
/prior_transition/trajectory_ready
```

Before each plan request, the runner sends a separate state-synchronization
request. The planner publishes the real-robot arm state to `/joint_states`,
then executes the same state as a `dual_arm` MoveIt joint goal on mock
hardware. The follower hand is opened separately. Only after this execution
finishes does the planner return the synchronization response and accept the
plan request. This makes the visible RViz state match the planning state.

## Run Experiment

The runner prepares and prompts for a new ORT open-loop-ramp episode before
each attachment attempt. Then run:

```bash
python3 /home/tp2/Documents/mios-wiring/python/shape_control_clip_fixing_prior.py \
  --scene-file /home/tp2/ws_humble/scene/trans/trans_clip_2_adapt_z.scene \
  --detect
```

The default clip order is `5 6 7 8`. Clip 5 is the initial direct Cartesian
setup; clips 6-8 use the single-arm planning workflow. The scene file is read in the same format as the transaction workflow. With
`--detect`, the top Kinect supplies the current DLO grasp point; without it,
the follower goal is placed 0.06 m from the measured leader goal toward the
previous fixture center, using the scene clip positions.

At startup, both MIOS robots are moved to the `home` named states from the
MoveIt SRDF, so homing does not depend on a trajectory file. Clip 5 then uses
the synchronized `real_world_task_5_stage_4_traj_1.txt` movement from the
transactions workflow to avoid independent-arm collision during initial setup.
Use
`--skip-homing` only when the robots are already positioned correctly.

The runner performs:

```text
clip 5 synchronized setup; compute and execute leader target for later clips
top-camera grasp detection
single-arm follower planning
r/y/q trajectory review
MoveIt and follower trajectory execution after accept
ramped-force attachment
```

Press `y` to execute the candidate trajectory, `r` to replan, or `q` to stop.
Replanning sends a new complete request for the same task and stage. The new
candidate overwrites the previous unaccepted follower trajectory file.

For transition planning and execution without attachment:

```bash
python3 /home/tp2/Documents/mios-wiring/python/shape_control_clip_fixing_prior.py \
  --only-motion-planning
```

This still executes the leader transition and accepted follower plan, but skips
grasping, ORT startup, and ramped-force attachment.

For unattended execution:

```bash
python3 /home/tp2/Documents/mios-wiring/python/shape_control_clip_fixing_prior.py \
  --detect \
  --auto-accept
```

The communication test also performs a replan by default. Use
`--no-replan` to test only one planning request.
