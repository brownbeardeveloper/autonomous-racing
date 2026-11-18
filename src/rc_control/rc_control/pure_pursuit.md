# pure_pursuit_node

Trajectory tracking for a 1/10 scale racecar. Reads a closed-loop raceline
from CSV and publishes `AckermannDriveStamped` on `pure_pursuit/drive` —
the same output contract as `follow_node`, which it takes over from.

Consumes the `racingline.csv` that the `map_pipeline` notebook writes, as is. Also
reads TUM `global_racetrajectory_optimization` output and plain `x,y,v`
files.

---

## Layout

```
rc_control/
├── rc_control/
│   ├── lib/pure_pursuit.py      # geometry + loading, no ROS imports
│   └── pure_pursuit_node.py     # the node
├── config/basic.yaml            # pure_pursuit_node section
├── launch/control.launch.py
└── test/test_pure_pursuit.py
```

The split is deliberate: everything in `lib/` runs without a workspace, so
the tracking maths can be tested in seconds on a laptop rather than only on
the car. `test_pure_pursuit.py` covers loading, geometry, speed shaping,
and a closed-loop kinematic bicycle simulation.

```bash
colcon build --packages-select rc_control
colcon test --packages-select rc_control --event-handlers console_direct+

# or, without a workspace at all:
python3 -m pytest rc_control/test/test_pure_pursuit.py -v
```

---

## Running

```bash
ros2 launch rc_bringup car.launch.py mode:=racing map:=<track>   # rc_maps/tracks/<track>/racingline.csv
ros2 launch rc_bringup sim.launch.py mode:=racing map:=spielberg
```

Requires a localiser publishing `map -> odom` (on the car: rc_localization's
particle filter). Without one, see `use_tf` below.

Live tuning:

```bash
ros2 param set /pure_pursuit_node speed_gain 0.6
ros2 param set /pure_pursuit_node lookahead_gain 0.45
```

Parameters bound at startup — topics, frames, `waypoints_file`, `wheelbase`
— are rejected with a reason rather than silently ignored.

---

## Why it is built this way

Three choices differ from a textbook pure pursuit, each for something that
shows up on a real car.

### Pose comes from TF, not from the odometry message

A raceline optimised on a SLAM map is expressed in the **map** frame. Wheel
odometry is expressed in **odom**, which drifts away from map over a lap.
Feeding an odom-frame pose into map-frame waypoints is a slow, silent
divergence: tracking looks fine at first and gets worse every lap, and the
symptom — drifting wide in the same corner each time — looks like a tuning
problem rather than a frame problem.

So the node looks up `map -> base_link` through TF. Odometry is still
subscribed for its velocity, which is what the lookahead scaling needs.

Set `use_tf: false` for a path recorded in the odom frame without a
localiser. The path then shares the drift and the two stay consistent.

### Commands go out on a timer, and a stale pose brakes

Publishing from the odometry callback ties control rate to sensor rate; a
stuttering odom source produces a stuttering drive command. The node runs a
fixed-rate timer instead, `control_rate` default 50 Hz.

That also makes the failure mode explicit. If the pose goes stale — TF
dropped, localiser died — the node commands zero rather than going quiet
and relying on a downstream watchdog to notice. Same on shutdown: a brake
is published on the way out instead of leaving the last command standing.

### Speed accounts for what is ahead

Pure pursuit is a steering law. It has no idea a corner is coming, so a car
that tracks its speed profile pointwise arrives at the apex still carrying
the straight's speed. So a speed profile is built once per path, and again
when a speed parameter changes, in this order:

| Step | Purpose |
|---|---|
| the file's profile × `speed_gain`, capped at `max_speed` | what the planner asked for, at a speed this car reaches |
| `sqrt(max_lateral_accel / \|kappa\|)` | grip limit, from the CSV's kappa column |
| backward pass `v_i <= sqrt(v_(i+1)^2 + 2 max_decel seg_i)`, twice round the loop | start slowing before the corner, not in it |

At runtime the profile is sampled at the car's position, then tapered with
the steering angle to ease off when tracking error has the wheel turned.

The braking pass is the one that matters most for a hand-recorded lap,
where the logged speed reflects how fast someone happened to drive rather
than what the tyres allow. Scaling by `speed_gain` before it, not after,
keeps `max_decel` the deceleration actually planned for.

### One more: the nearest-point search is windowed

Searching the whole path each cycle sounds harmless and is not. A hairpin
or a pit straight puts two topologically distant parts of the line within a
metre of each other, and a global search will snap to the wrong one —
teleporting the target ahead or behind by half a lap. The search is seeded
from the previous match and limited to `search_window` waypoints either
side. A global search is used only on the first cycle and after a recovery.

`search_window` must comfortably exceed the waypoints covered per control
step, and stay well under half the loop. At 1 m spacing, 50 Hz and 8 m/s
that is 0.16 waypoints per step, so the default of 40 has enormous margin.

---

## Parameters

### Localisation

| Name | Default | Notes |
|---|---|---|
| `use_tf` | `true` | Pose from TF. False = trust the odom pose. |
| `path_frame` | `map` | Frame the waypoints are in. |
| `base_frame` | `base_link` | Rear axle, ideally — pure pursuit's reference point. |
| `control_rate` | `50.0` Hz | Drive command rate. |
| `pose_timeout` | `0.3` s | No fresh pose for this long → brake. |

### Vehicle

| Name | Default | Notes |
|---|---|---|
| `wheelbase` | `0.33` m | Front-to-rear axle. Measure it. |
| `max_steering_angle` | `0.36` rad | Servo mechanical limit. |

### Lookahead

`Ld = clip(lookahead_distance + lookahead_gain * v, lookahead_min, lookahead_max)`

| Name | Default |
|---|---|
| `lookahead_distance` | `1.0` m |
| `lookahead_gain` | `0.3` m per m/s |
| `lookahead_min` / `lookahead_max` | `0.5` / `3.0` m |

Too short oscillates, too long cuts corners. Tune in that order: raise the
base until the wobble on a straight stops, then raise the gain until fast
corners stop being cut.

### Speed

| Name | Default | Notes |
|---|---|---|
| `speed_gain` | `1.0` | Start at 0.5 on a new track. |
| `max_speed` | `4.0` m/s | Hard clip, independent of `speed_gain`. |
| `min_speed` | `0.0` | Floor once moving; 0 disables. |
| `max_lateral_accel` | `6.0` m/s² | 0 disables the curvature cap. |
| `max_decel` | `3.0` m/s² | Braking the profile plans for; 0 disables. |
| `steering_speed_factor` | `0.4` | Speed retained at full lock; 1.0 disables. |

### Tracking

| Name | Default | Notes |
|---|---|---|
| `search_window` | `40` | Waypoints either side of the last match. |
| `max_path_error` | `2.0` m | Beyond this, stop instead of steering back blind. |
| `publish_debug` | `true` | Path, lookahead point, diagnostics. |

---

## Topics

| Topic | Type | Direction |
|---|---|---|
| `pure_pursuit/drive` | `AckermannDriveStamped` | out |
| `odom` | `Odometry` | in |
| `/tf`, `/tf_static` | — | in, when `use_tf` |
| `~/path` | `nav_msgs/Path` | out, latched for late RViz |
| `~/lookahead_point` | `PointStamped` | out |
| `~/diagnostics` | `Float32MultiArray` | out |

Diagnostics layout: `[cross_track_error, path_error, lookahead, steer, cmd_speed, measured_speed]`.
Cross-track error is signed, positive when the path lies to the left.

```bash
ros2 topic echo /pure_pursuit_node/diagnostics
```

---

## Bringing it up on a new track

1. Map the track, set `TRACK` in the `map_pipeline` notebook and run it: it
   writes the track's `racingline.csv`.
2. Launch with `speed_gain: 0.4` and confirm in RViz that `~/path` lands
   on the track. If it does not, the frames are wrong — that is a
   localisation problem, not a tuning one, and no amount of lookahead will
   fix it.
3. Watch cross-track error on `~/diagnostics`. Steady oscillation on a
   straight means the lookahead is too short.
4. Raise `speed_gain` in steps. When the car starts running wide in fast
   corners, either the lookahead is too long or `max_lateral_accel` is
   above what the tyres deliver.
5. `max_lateral_accel` is the honest limit to find by experiment. Set it
   from measurement, not optimism — the planner's velocity profile is only
   as good as this number.

## Known limits

- **Loops only.** Point-to-point paths are not supported: the last waypoint
  wraps to the first. A path with a real start and end needs a terminal
  state that stops the car, which this node does not have.
- **No obstacle awareness.** It follows the line and nothing else. Keep the
  safety node upstream of the motor.
- **No reverse.** Negative speeds are clipped to zero; recovering from a
  spin is out of scope.
- **Rear-axle reference.** Pure pursuit's geometry assumes `base_frame` is
  the rear axle. A base link at the vehicle centre introduces a small,
  systematic corner-cutting bias — usually tolerable at 1/10 scale, but it
  is a bias, not noise.
