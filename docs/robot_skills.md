# Robot skill CLI

Start the workcell first (restart it if the object-pose bridge or joint limits
have changed):

```bash
cd ~/ur3
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3e_llm_control ur3e_workcell.launch.py
```

In another terminal, source the same environment and run one skill:

```bash
ros2 run ur3e_llm_control robot_skill.py home
ros2 run ur3e_llm_control robot_skill.py place green
ros2 run ur3e_llm_control robot_skill.py pick green zone_Intermediate
```

Run these separately, waiting for each command to finish.

- `home`: return the arm to the initial joint angles from
  `config/initial_positions.yaml`; keep a held object held.
- `place OBJECT`: grasp, lift, and hold the object, following the requested API.
- `pick OBJECT ZONE`: grasp if needed, transfer, release, and retract.

Objects accept `red`, `yellow`, `green`, `blue`, `purple` or model IDs such as
`green_box`. Zones accept `gray`, `red`, `green`, `yellow` or `zone_Intermediate`,
`zone_1_red`, `zone_2_green`, `zone_3_yellow`.

The node runs one requested function, logs its outcome, and exits (0 for
completion, 1 for execution failure, 2 for CLI syntax errors). A closing command
remains active in the gripper controller after `place` exits. The next invocation
recovers the held object's identity from MoveIt's planning attachment and checks
its actual position in Gazebo. Do not run multiple skill commands simultaneously.

`self.objects` is an array containing object IDs, colours, dimensions and current
positions. Initial values come from `workcell.world`; current values update from
`/world/ur3e_workcell/dynamic_pose/info`. No camera data is used. Skills still use CLI arguments rather than a command topic.
Final execution events are sent to `/robot_skill/events` for task_validator.

Collision/limit configuration belongs in `config/`:

- `ur3e_joint_limits.yaml` sets the base's URDF interval to -135..135 degrees.
- `joint_limits.yaml` supplies matching MoveIt limits and acceleration limits.
- `ur3e_workcell.srdf` retains narrow physical-neighbour collision exclusions.
- `ompl_planning.yaml` configures planning resolution and timing tolerance.
- `robot_skill.yaml` sets grip force, timing, clearance and motion speed.

`robot_action.py` is the MoveIt helper module used by the skill node, not a
separate node. It accounts for the table, legs and current boxes, checks robot
self-collision, prefers IK solutions near current joint angles, prechecks vertical
approaches, and executes complete collision-checked trajectories. The base range
prevents full-circle turns; it does not guarantee a globally shortest path around
obstacles. Only the selected object's finger contact and the physical table/base
mount are exempted. Planning attachments account for payload size; Gazebo holds
objects by friction, without physical attachment.


## Task validator

Start the validator in a sourced terminal **before** running skill commands:

```bash
ros2 run ur3e_llm_control task_validator.py
```

Read results in another sourced terminal:

```bash
ros2 topic echo /task_validator/result
```

Examples:

```text
home success
place green success
pick green zone_Intermediate success
place pink failed INVALID_OBJECT
pick red zone_1_red failed PLANNING_FAILED
```

Unknown object/zone names produce `INVALID_OBJECT`. A `pick` destination already
occupied by a box produces `PLANNING_FAILED` before grasping or moving the arm.
Occupancy is checked from live Gazebo object positions, without using a camera.
`robot_skill` publishes one final result after actual execution/verification;
`task_validator` formats it as the skill name, its original CLI parameters,
then success/failed and error code. `home` has no parameters. Object and zone
aliases are preserved exactly as entered, including invalid names.
Start the validator first because a CLI publisher exits after its one command.
The validator retains its latest text result while running.

## Action queue

Start the workcell and task validator, then start the queue in another sourced
terminal:

```bash
ros2 run ur3e_llm_control action.py
```

Send a single CLI skill or a sequence separated by semicolons/newlines to
`/action/command` (`std_msgs/msg/String`). Syntax is the same as robot_skill:

```bash
ros2 topic pub --once /action/command std_msgs/msg/String \
  "data: 'home; place green; pick green zone_Intermediate'"
```

Additional messages append to the queue. To start execution:

```bash
ros2 topic pub --once /action/command std_msgs/msg/String "data: 'action'"
```

After `action`, input is ignored. Skills execute one at a time as robot_skill
CLI processes. The queue advances only after the process exits successfully
and task_validator confirms success for that invocation. Request IDs carried
on `/task_validator/events` distinguish repeated commands and unrelated results;
the user-facing `/task_validator/result` text format stays the same.

An `action` trigger with an empty queue is rejected and leaves the node waiting
for skills. For the LLM workflow, llm_control sends a checked plan, starts
execution automatically, and waits for completion. Action also checks the fresh
world state before starting and after each skill. See the
[checked LLM workflow](../README.md#llm-control-with-checked-plans); no manual
`action` trigger is needed there.

Start the downstream task_validation subscriber before triggering execution.
This package publishes the task result on `/task_validation`
(`std_msgs/msg/String`); it does not create a task_validation node:

```bash
ros2 topic echo /task_validation
```

Any validator failure stops the remaining queue, publishes `task failed` and
exits action with code 1. Exhausting the queue publishes `task success` and
exits with code 0. If the validator
is absent for 10 seconds, a skill times out (default 300 seconds), or a process
exits without successful validation, the task fails. Adjust the per-skill
timeout with `--ros-args -p skill_timeout:=300.0`.

Task result delivery is acknowledged before action exits. The subscriber must
already be running: this one-shot publisher cannot retain messages after exit.
Restart action.py to submit a new task.
