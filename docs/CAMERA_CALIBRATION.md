# Four-foot camera calibration

The physical Orin scenario provides `/camera/calibration/start`, `/cancel`, and
`/save` services (all `std_srvs/srv/Trigger`). It never commands motors or starts
the policy. `enable_camera_calibration:=false` disables this feature.
The calibration node subscribes to depth only while collecting. The physical
bridge also uses depth for the live ground reference described below.

This calibration compares the floor observed by the depth camera with the plane
predicted from all four feet using the selected saved model and Foundation joint
feedback. It estimates **camera roll, pitch and body-frame Z** conditional on the
existing X, Y and yaw. A single unmarked floor cannot identify translation along
the floor or rotation about its normal. Measure X/Y and heading separately; those
values are retained, not silently treated as calibrated.

The quantity being calibrated is the camera's **mounting height relative to the
body origin**, not its distance to the floor. The standing feet and joint angles
establish body-to-floor height. Depth establishes camera-to-floor height; comparing
the two solves the mounting Z, accounting for body tilt. A tape measurement is
not required. The default `camera_ground_height` is zero, selecting depth.

## Live model-ground alignment

`estimate_ground_from_depth:=true` connects the detected floor to both
`map -> robot/world` and the policy's base-height calculation. The first reliable
depth plane is required before physical feedback is published. While the Pi
confirms the policy is stopped and no script is running, the bridge refines the
reference at up to 2 Hz. This includes moving the gray model floor when the depth
floor changes relative to the tracking origin. The same height correction is
applied to the robot pose, so TF and feedback remain consistent.

`/camera/ground/reference` is a typed `PoseStamped` for the horizontal model floor
under the body, in the gravity-aligned world frame. `/camera/ground/status` reports
`tracking`, `waiting`, `holding` (last valid floor retained after a rejected depth
sample), or `frozen`. Once initialized, the reference is frozen while policy is
active or stopped-state confirmation is unavailable. Invalid or stale depth
never replaces a valid floor. Calibration preview/save controls the mount;
background ground alignment does not require saving a mount or restarting after
each detection.

The current model/contact estimator assumes approximately level ground. The
fitter rejects walls and steep surfaces and preserves gravity-aligned axes; it
sets the horizontal floor height at the body's position rather than rotating the
policy's world frame to match a ramp. This is not a terrain map or force sensor.

An optional measured override remains available: disable depth grounding with
`estimate_ground_from_depth:=false` and set a positive `camera_ground_height` at
`camera_height_frame`. A single measurement establishes the reference once at
startup and is also used for mount calibration. With both depth grounding and the
measurement disabled, `ground_height_offset` supplies the legacy fixed reference.

## Operator procedure

1. Verify joint homing/zeros and leg geometry. Put all four feet on one flat,
   level floor with a broad patch of that floor visible to the camera. Avoid
   steps, reflective floors and obstacles. The four-foot stance does not require
   a perfectly level body. Encoder errors and body tilt over 20 degrees prevent
   calibration. A mathematically coplanar set of feet cannot prove all four feet
   carry weight: the operator must verify contact.
2. Stop the policy and finish any motor scripts. Keep the existing physical
   camera/feedback launch running. Leave the robot stationary in a supported
   stance; this feature neither arms nor disarms it.
3. Call start and keep the robot still. It collects at least 15 distinct depth
   frames over at least three seconds, with a 15-second deadline:

   ```bash
   ros2 service call /camera/calibration/start std_srvs/srv/Trigger '{}'
   ```

4. Review `/camera/calibration/status` in Foxglove. `preview` means a consistent
   candidate exists. `/camera/calibration/mount` is a typed `PoseStamped` showing
   the candidate camera-link pose relative to the model body.
   `/camera/calibration/heights` is a typed `Vector3Stamped`: X is vertical
   camera-to-floor height in meters, Y is body-origin-to-floor height, Z is floor
   fit RMS error. These are calibration-time measurements, not live terrain
   estimates. The status report also identifies retained components, sample
   count, residual, consensus, model-dependent assumptions, and capture time.
5. Save only after review, or cancel to discard the candidate:

   ```bash
   ros2 service call /camera/calibration/save std_srvs/srv/Trigger '{}'
   # or:
   ros2 service call /camera/calibration/cancel std_srvs/srv/Trigger '{}'
   ```

   The file defaults to `~/.config/catbot/camera_calibration.json` **on Orin**.
   `camera_calibration_output` overrides the path. Saves are atomic. Saving
   does not change the running feedback transform and never restarts tracking.
6. Load the saved mount on the next physical launch:

   ```bash
   ros2 launch catbot_launcher physical_orin.launch.py \
     workspace:=/mnt/nvme/workspace/projects/ros2_ws \
     camera_calibration_file:=/home/catbot/.config/catbot/camera_calibration.json
   ```

   An explicitly supplied calibration file overrides the launch mount values.
   Invalid files or changed saved robot geometry fail launch rather than silently
   falling back. Recalibrate after moving the camera or changing joint zeros.

## What is and is not calibrated

Depth uses `depth_topic` and the matching rectified `camera_info_topic`; the
default ZED optical frame is transformed through the camera's factory TF into
`camera_frame` (`zed_camera_link`). The existing robot TF supplies body-relative
foot centers using encoder-derived kinematics, avoiding a duplicate kinematics
implementation. Sphere radii are included when locating the contact plane.

Samples require fresh, time-aligned measurements, healthy encoders/heartbeats,
small joint velocities and gyro rates, no active policy/scripts, and less than
0.5 degree joint drift. Plane fitting rejects walls, narrow/degenerate samples,
poor consensus and large corrections. Sample variation above 0.5 degree tilt or
5 mm mount Z prevents a preview. These are consistency bounds, not an accuracy
guarantee: wrong joint zeros, flex, backlash or leg dimensions bias the result.

The report includes `ground_height_offset_at_capture`, the vertical correction
for that world/tracking origin. It is not saved as a permanent mount parameter;
the live depth reference handles current ground alignment independently. This
feature does not reset camera tracking or command any actuator. Saved mount
corrections still require explicitly loading the calibration file on launch.

For complete six-degree camera placement, add a surveyed target or deliberately
varied known poses with enough independent plane orientations and a separate
observability check. Repeating the same level stance improves averaging but
does not reveal X/Y/yaw.
