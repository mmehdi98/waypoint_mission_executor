# waypoint_mission_executor

A ROS 2 action server that drives a robot through a list of waypoints. Each
waypoint carries a target pose, a speed limit, and a zone radius, mirroring
the argument structure of industrial move instructions (ABB `MoveL` with
`speeddata` and `zonedata`).

The plant is turtlesim. The node touches the robot only through a
`geometry_msgs/Twist` command and a pose subscription, so the control logic
is not tied to turtlesim beyond those two message types.

## Packages

| Package | Contents |
| --- | --- |
| `waypoint_interfaces` | `RobTarget.msg`, `MoveInstruction.msg`, `MoveAlongPath.action` |
| `turtle_handler` | `turtle_controller` (server), `mission_client` (example client), `controller` (ROS-free control library) |

## Interfaces

```
# RobTarget.msg
float32 x
float32 y
float32 theta

# MoveInstruction.msg
RobTarget target
float64 velocity     # speed limit for this segment
float32 zone_data    # blend radius; 0 means stop at the target

# MoveAlongPath.action
MoveInstruction[] path
---
string message
---
RobTarget current_target
RobTarget current_position
float32 distance_to_next
```

A whole path is one goal. Goals are queued and run in order.

## How the server works

Everything runs in the pose subscription callback, which fires at the
turtlesim publish rate (~62 Hz). One callback does three things in order:
advance the mission state, compute a velocity command, publish it. There are
no worker threads and no mutexes, and the goal handle is only ever touched
from that one callback.

- `goalCallback` validates every instruction in the path before accepting:
  non-empty path, target inside the 0–11 workspace, velocity above 0.01,
  zone data in [0, 5]. A bad instruction rejects the whole goal.
- `acceptedCallback` pushes the goal handle onto a queue and returns.
- `motionPlanner_` pops the next goal when idle, walks `path_index_` forward,
  publishes feedback every 200 ms, and calls `succeed()` or `canceled()` on
  the handle. It loops rather than returns after a terminal state, so a
  finished goal and the start of the next one happen in the same tick.
- A waypoint is considered reached when the remaining distance drops below
  `max(zone_data, 0.01)`. With a non-zero zone the next instruction becomes
  active before the current target is reached, so the robot cuts the corner
  instead of stopping on it.
- Cancellation is checked once per tick, so a cancel request takes effect
  within about 16 ms.

## Controller

`turtle_handler::computeVelCmd` in `src/controller.cpp` is plain C++ with no
ROS headers. It takes a `Pose` and a `MoveInstruction` struct and returns a
`Command`. Conversions between these structs and the ROS messages live in
`include/turtle_handler/conversions.hpp`, which is the only file in the
control path that includes ROS.

Linear velocity is proportional to the distance error, scaled by
`cos²(e/2)` on the heading error and clamped to the instruction's speed
limit. That scaling drives the linear velocity to zero when the target is
behind the robot, so it turns first instead of driving off in the wrong
direction. Angular velocity is a pursuit term proportional to the current
linear velocity plus a proportional term on the heading error.

Keeping the controller ROS-free is what makes the test harness below
possible: it compiles with a single `g++` call, with no workspace, no
`colcon`, and no running graph.

## Build

```bash
cd <your_ws>
colcon build
source install/setup.bash
```

Built and tested on ROS 2 Lyrical.

## Run

Three terminals, each with the workspace sourced:

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtle_handler turtle_controller
ros2 run turtle_handler mission_client
```

`mission_client` sends a hardcoded three-waypoint path and logs feedback.
To send your own path without recompiling:

```bash
ros2 action send_goal /move_along_path waypoint_interfaces/action/MoveAlongPath \
  "{path: [{target: {x: 8.0, y: 4.0, theta: 0.0}, velocity: 1.0, zone_data: 0.5}]}" --feedback
```

## Test harness

`test/controller_harness.cpp` runs the controller against a forward-Euler
model of the turtle, with no ROS involved:

```bash
cd turtle_handler
g++ -std=c++20 -Wall -Wextra -Iinclude test/controller_harness.cpp src/controller.cpp -o harness
./harness
```

Each case is scored on path ratio (distance travelled over straight-line
distance), time ratio against the ideal at the commanded speed, control
effort `∫ω²dt`, angular acceleration cost `∫α²dt`, peak ω and α, the
fraction of steps at the speed limit, and the number of steps that increased
the distance to the target. The harness returns the number of failed checks
as its exit code.

## Limitations

- The target's `theta` is carried through the interfaces but the controller
  ignores it. Only position is tracked; final heading is whatever the
  approach direction leaves behind.
- Linear velocity is never negative. The robot always approaches head-on and
  turns in place rather than reversing.
- Controller gains are hardcoded. They are not ROS parameters yet.
- No abort path: a goal that cannot make progress runs until it is cancelled.
- No planner and no obstacle avoidance. Targets come from the client.
- The workspace bounds in `validateInstruction_` are turtlesim's 0–11 square.
