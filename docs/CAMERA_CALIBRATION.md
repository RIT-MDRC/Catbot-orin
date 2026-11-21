# Four-foot camera calibration

The physical Orin scenario provides `/camera/calibration/start`, `/cancel`, and
`/save` services (all `std_srvs/srv/Trigger`). It never commands motors or starts
the policy. `enable_camera_calibration:=false` disables this feature.
Depth-image subscriptions exist only while a capture is collecting.

This calibration compares the floor observed by the depth camera with the plane
predicted from all four feet using the selected saved model and Foundation joint
feedback. It estimates **camera roll, pitch and body-frame Z** conditional on the
existing X, Y and yaw. A single unmarked floor cannot identify translation along
the floor or rotation about its normal. Measure X/Y and heading separately; those
values are retained, not silently treated as calibrated.

The quantity being calibrated is the camera's **mounting height relative to the
torso/body origin**, not its distance to the floor. The standing feet and leg
angles establish body-to-floor height. The supplied camera-center-to-floor
measurement (currently 0.43 m) establishes the other height; comparing the two
solves the mounting Z, including body tilt and the factory camera-center offset.
For a level body this is simply camera-ground height minus body-ground height.
The depth plane still supplies orientation and a separate distance cross-check.
With `camera_ground_height:=0.0`, depth supplies the distance instead. The report
labels `height_source`, `measured_point_height_m` and `depth_camera_height_m`.
Use the tape measurement only while the robot remains in the measured stance.

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
for the **current** world/tracking origin. It is not loaded as a permanent mount
parameter: restarting tracking changes that origin. The existing
`camera_ground_height` launch setting defaults to the measured **0.43 m**, from
the midpoint between the lenses (`zed_camera_center`) vertically to the floor.
At the first valid physical snapshot, this point's world Z is used to establish
the ground plane: `ground_height_offset = camera_ground_height - camera_world_z`.
Both the model TF and policy height use that same offset. Stand the robot at the
measured camera height before starting the physical scenario. Later motion does
not re-anchor the ground or force the camera to remain at 0.43 m. The camera's
factory center-to-mount transform is included; this is not a body-height or mount
Z setting. `camera_height_frame` selects a different measured reference point.
Set `camera_ground_height:=0.0` to use `ground_height_offset` directly instead.
This feature does not add
a continuous terrain/height estimator, measure foot force, change policy inputs,
or reset/rotate the gravity-aligned world to match the floor.

For complete six-degree camera placement, add a surveyed target or deliberately
varied known poses with enough independent plane orientations and a separate
observability check. Repeating the same level stance improves averaging but
does not reveal X/Y/yaw.
