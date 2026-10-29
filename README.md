# SentryMaster 2026 — Autonomous Sentry Robot

Autonomous swerve-drive robot for **SentryMaster 2026**, the freshman algorithm competition of the HITSZ RoboMaster team (南工骁鹰). The robot plays a full 5-minute match on its own: it drives between scoring zones, holds position, and spins in place where the rules require it.

Built on ROS 2 Kilted (Ubuntu 24.04). Based on the organizers' workspace [QxhhhQ/SentryMaster2026](https://github.com/QxhhhQ/SentryMaster2026); my work is on the `dev` branch.

## Architecture

```
decision (Python)  ──/decision_to_planner──▶  planner (C++)  ──/planner_to_driver──▶  uart_driver (C++)  ──serial──▶  chassis
       ▲                                           ▲
   decision.yaml                         /odin1/odometry_highfreq (Odin1 lidar odometry)
```

- **decision** — reads waypoints and a task list (`move` / `stay`) from YAML and sends the current target to the planner.
- **planner** — drives straight toward the target, converts field-frame velocity into robot-frame velocity, holds heading with a P-controller, and spins on command.
- **uart_driver** — converts m/s and rad/s into the chassis protocol and sends it over the serial port.

Custom messages in `rm_msgs`: `DecisionToPlanner` (x, y, is_spinning), `PlannerToDriver` (vx, vy, wz), `DecisionToDriver` (posture).

## What I did

- Designed the message interfaces and the decision → planner → driver pipeline
- Wrote the planner: goal tracking, field-to-robot frame rotation, spinning mode, heading hold
- Wrote the decision node: waypoints and task sequence configured from YAML
- Fixed the workspace for ROS 2 Kilted (dependencies, launch issues)
- Calibrated on the real field: Odin1 is mounted upside down, so I added `yaw_offset` and `field_rotation`; measured the field and tuned the route
- Added `run.sh` for runs with logs and rosbag recording

## Result

The robot drove the full route autonomously on the official field — one of only two teams whose robot moved on its own. After the competition I was accepted into the team's Sentry group.

## Build and run

```bash
./scripts/build.sh      # or: colcon build --packages-select <pkg>
./scripts/chassis.sh    # start
./scripts/run.sh        # start with logs + rosbag (~/sentry_logs/<date>/)
```

Main parameters: `src/sentry_planner/config/global_planning.yaml`, route and tasks: `src/sentry_decision_sys/config/decision.yaml`.

## Credits

- Workspace template and competition: [QxhhhQ/SentryMaster2026](https://github.com/QxhhhQ/SentryMaster2026)
- `odin_ros_driver` — © Manifold Tech Ltd., Apache License 2.0
