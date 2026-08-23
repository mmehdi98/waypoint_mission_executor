# waypoint_mission_executor

## What it does
Moves a robot to a sequence of targets specified by (x, y, theta). Motion
requests are submitted to a queue and performed in order. Each request can
specify the velocity and zone data. The current backend is turtlesim; the
node interacts with the robot only through velocity commands and pose
feedback.

## Architecture
The server node controls robot motion. It exposes an action for submitting
motion requests which acknowledges the goal on arrival, publishes feedback
while the motion runs, and returns a result once the queue reaches and
completes it. The action also allows a client to cancel a goal, whether it
is queued or already executing. Starting and stopping the robot remain
services, since both complete immediately.

The client nodes submit motion requests to the server by specifying an
array of motion objects. A motion object is composed of a target, a
velocity, and zone data, mirroring the argument structure of industrial
move instructions.

## Interface design
...

## Zone data
By specifying zone data, the robot will start the next motion object
before finishing the current motion object, resulting in a more fluid
motion through the path while reducing precision. It is a similar idea to
that implemented in most industrial manipulators, such as the zonedata in
ABB robots.

## Known limitations
- Negative linear velocity is rejected. The controller only approaches
  targets head-on.
- No obstacle avoidance and no path planner. Targets are supplied by the
  client, not computed.

## Usage
...
