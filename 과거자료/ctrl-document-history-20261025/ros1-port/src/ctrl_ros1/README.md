# ROS1 execution boundary

This package supplies only the APIs the copied competition nodes use. It does
not change planning, perception, localization, control, or UDP calculations.
Production imports explicitly use `ctrl_ros1`; no package named `rclpy` is
installed or injected into a production process.

## Callback dispatch

`rospy.Subscriber` callbacks only append incoming messages to the subscription's
bounded `KEEP_LAST` queue. Application callbacks run in `SingleThreadedExecutor`
unless the original entry point selects `MultiThreadedExecutor(num_threads=2)`.
The default callback group is mutually exclusive. The behavior node retains its
independent mutually exclusive events group and original `event_lock`.

`ctrl_ros1.ctrl_tf2_ros.TransformListener` also enqueues `/tf` and `/tf_static`
through the obstacle node's executor, preserving the Humble listener's
reentrant group and depth 100. It uses Noetic's native TF buffer without adding
a frame-graph service. This keeps TF buffer updates serialized with cloud
callbacks in the obstacle node's single-threaded executor.

A readiness snapshot traverses timers before subscriptions, each in creation
order, taking one callback per ready entity per pass. Messages are taken when
execution begins, so a depth-one footprint subscription uses the newest message
present then. Application exceptions propagate to the entry point rather than
being swallowed by a TCPROS thread. This follows the relevant
[Humble executor traversal](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/executors.py).

Concurrency between different groups remains subject to operating-system thread
scheduling, as in the source's two-thread executor. This is not evidence that
two independent live runs will take identical callbacks at identical instants.
Use an identical callback trace and decision timestamps for differential replay.

## Time

Node clocks read `rospy.Time.now()`. `Time.to_msg()` and `Duration.to_msg()` produce
native ROS1 time/duration values; calculations retain integer nanoseconds. The
copied nodes use ROS1 `secs`/`nsecs` field spelling, with units unchanged.

Timers become ready from the node ROS clock, including `/clock` pause, forward
jumps and backward jumps. A backward jump before a timer's last call schedules
its next call one period after the new time. Missed periods advance the deadline
using the same arithmetic and equality rule as
[Humble rcl_timer_call](https://github.com/ros2/rcl/blob/humble/rcl/src/rcl/timer.c).
Wall-clock condition waits only wake the executor; they never advance ROS time.
An operating-system wall-clock adjustment when simulation time is inactive
does not inject an additional ROS clock-jump reset into the timer.
The original code's independent `time.monotonic()`, `time.time()`, watchdog
threads, UDP periods and bounded shutdown handoff remain in the copied nodes.

ROS1 simulation time is controlled by global `/use_sim_time`. The source launch
does not set per-node simulation-time overrides. An explicitly declared
`use_sim_time` that disagrees with the global value raises an error. Changing
the ROS1 clock source dynamically after node initialization is unverified.

## Parameters and resources

Launch sets private `~_ros2_parameter_files` to the unchanged source YAML file
or an ordered list of files. Node initialization reads matching wildcard and
node-name sections and flattens nested dictionaries. Precedence, from low to
high, is declaration default, YAML files, private ROS1 server values, CLI
parameter files and CLI values, constructor `parameter_overrides`.

Python names such as `roundabout.enabled` remain unchanged. Private ROS1 names
encode periods as slashes (`~roundabout/enabled`); launch uses the same encoding.
Parameter values preserve their Python types and an override that violates the
declared type fails. Node lifetimes follow rospy's one-node-context-per-process
restriction; after `shutdown()` a new process is required.

`get_package_share_directory()` uses `rospkg.RosPack().get_path()`, which resolves
catkin source/devel and installed package resource locations.

## Point clouds

`ctrl_point_cloud2.py` vendors the
[official Humble implementation](https://github.com/ros2/common_interfaces/blob/2de367120bab435a5874fc69b96b348687509405/sensor_msgs_py/sensor_msgs_py/point_cloud2.py)
retrieved 2026-10-13, retaining its BSD license. The only adaptations are a direct
NumPy recfunctions import (NumPy >= 1.16; Ubuntu 20.04 provides a newer version)
and native `bytes` output for Noetic genpy's uint8 serializer, plus a writable
buffer for in-place byte swapping of ROS1-deserialized bytes.
Field offsets, field order, dtypes, NaN handling, `is_dense`, organized reshaping,
validation assertions and serialization calculations are unchanged. The
upstream behavior of modifying a big-endian buffer during reads is retained.

The exact unmodified dependency fixture and its provenance are in
`test/fixtures/point_cloud2_humble.py` and `point_cloud2_provenance.json`.
Commit: `2de367120bab435a5874fc69b96b348687509405`; file SHA256:
`79582d403a689d88468e064dcfc39a4d6748de3459bc0492084a4abb0831e7eb`.
The source deployment's installed `sensor_msgs_py` version is unknown. Compare
that installed file and package version before claiming dependency equivalence.

## Generated message defaults

Humble's [Quaternion definition](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Quaternion.msg)
defaults `w` to 1; [Noetic's definition](https://github.com/ros/common_msgs/blob/noetic-devel/geometry_msgs/msg/Quaternion.msg)
has an implicit zero default. `ctrl_ros1.ctrl_messages.Marker` subclasses the
native Noetic Marker and restores `pose.orientation.w=1` only when the caller
omits `pose`, including DELETEALL markers. Explicit poses, all other defaults,
constants, message type/MD5 metadata and inherited serialization remain native.
No ROS message class is monkeypatched. A full native wire roundtrip test is
included and skips explicitly when actual Noetic messages are unavailable.

## Transport difference requiring a decision before equivalence sign-off

Sensor QoS requests `BEST_EFFORT` depth 5 in the source. rospy's TCPROS transport
is reliable; it cannot express DDS BEST_EFFORT, DDS discovery, DDS history or
deadline behavior exactly. The adapter retains the requested QoS metadata and
queue depth, and maps `TRANSIENT_LOCAL` publishers to ROS1 `latch=True`. A ROS1
publisher, receiver and kernel may buffer messages before the executor queue;
delivery and loss under congestion therefore need integration verification.
`rospy.Subscriber` retains its native 65,536-byte receive buffer setting;
PointCloud2 messages can exceed that size and span reads. Increasing it would
affect which backlog is dropped, so this port does not invent a replacement
buffer tuning value. Verify actual cloud sizes, TCPROS queue behavior and
latency on the target under load.
Latched DDS subscriptions also differ from volatile DDS subscriptions when
connecting to latched ROS1 publishers; all source competition subscriptions to
latched map/mode publishers request transient-local, but external tools may not.

The implementation currently uses native TCPROS. It does not claim to resolve
this difference. UDPROS is not offered by rospy; implementing a new UDP protocol
would change discovery, fragmentation and loss characteristics and would still
not reproduce the original DDS middleware. The alternatives are to accept and
validate TCPROS queue/delivery behavior for the deployment or retain the ROS2
sensor transport through a separately evaluated bridge. The user's decision is
required before treating live transport semantics as preserved.

## Verification

From the repository root on Windows or Linux:

```text
python -m unittest discover -s ros1-port/src/ctrl_ros1/test -v
```

These tests double only ROS1 transport/time and exercise the production
dispatcher. They cover readiness order, one-message traversal, KEEP_LAST,
take-at-execution, thread exclusion, the behavior events-group concurrency,
ROS-time pause/jumps, missed-period equality, parameter types/precedence,
native time conversion and latch/depth settings. Comparisons use exact values,
including exact integer nanoseconds; no numerical tolerance is applied.

With NumPy installed, the same command also compares the port's cloud helper
against the pinned upstream fixture: payload bytes, field order and dtypes,
NaNs and `is_dense`, empty clouds, big-endian mutation, organized shapes and
padding behavior, validation assertions, and the Noetic genpy-style uint8
packing operation. Values and payload bytes must match exactly (`atol=rtol=0`).
The upstream fixture's SHA256 and Git blob hash are checked before comparison.

The tests do not establish catkin build, actual ROS1 serialization, TF runtime,
ROS clock-source switching, network-load behavior, signal delivery or vehicle
driving behavior. Those require Ubuntu 20.04 / ROS1 Noetic integration runs and
matched original-versus-port input/callback replay.
