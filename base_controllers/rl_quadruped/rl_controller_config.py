# -*- coding: utf-8 -*-
"""Self-contained configuration for :mod:`rl_quadruped_controller`.

This module is the **only** configuration the RL controller reads.  It does not import
:mod:`base_controllers.params` and nothing here is merged with it: the controller hands this module
to :class:`BaseController` as its ``external_conf``, which replaces ``params.robot_params`` for the
lifetime of the process.  So a robot that runs this controller is described here, once, and tuning it
cannot disturb any other controller in the framework.

That is also why the robot description fields (``joint_names``, ``ee_frames``, ``q_fold``, ``ip``,
``spawn_*``) appear below even though ``params.py`` has its own copies - the two are deliberately
independent.  The URDF itself still comes from the ``<robot>_description`` package, since that is
where the kinematics live.

Resolution order, later winning:

1. :data:`DEFAULTS`
2. ``ROBOTS[robot_name]``
3. ``ROBOTS[robot_name]['real']``, only when running on hardware - **normally empty**

The control parameters are deliberately identical in simulation and on the robot: the policy was
trained against a model of the machine, so a gain or a duration that has to differ between the two is
a modelling error to fix rather than a difference to paper over.  The ``real`` mechanism is kept for
the case where something genuinely is hardware-only, but nothing uses it.

:func:`get_config` returns the merged dictionary and also publishes it as
:data:`robot_params`, in the shape :class:`BaseController` expects.

Gains are per-joint arrays in the joint order given by ``joint_names``.  ``ki`` never appears: the
controller runs no integral term in any state, by design - an integral term on a position-controlled
leg winds up while the foot is loaded and kicks on lift-off.
"""

import numpy as np

# Consumed by BaseController.__init__ via external_conf.
verbose = False

# Filled in by get_config(); BaseController reads spawn_*, joint_names and real_robot from here.
robot_params = {}


# Locosim's quadruped descriptions share one naming convention (lf/lh/rf/rh legs, haa/hfe/kfe
# joints), so the helpers below spell it out once.  A robot that names things differently writes its
# own lists in ROBOTS.
_LEGS = ("lf", "lh", "rf", "rh")


def _legs(haa, hfe, kfe):
    """A symmetric four-leg posture, mirrored on the right side, in ``joint_names`` order."""
    return np.array([haa, hfe, kfe,      # lf
                     haa, hfe, kfe,      # lh
                     -haa, hfe, kfe,     # rf
                     -haa, hfe, kfe])    # rh


def _gains(haa, hfe, kfe):
    """A per-joint gain array, the same triple on each of the four legs."""
    return np.array([haa, hfe, kfe] * 4)


def _scaled_gains(mass_ratio, length_ratio):
    """(kp, kd) ratios to scale a tuned robot's gains into a starting point for another robot.

    kp holds a posture against gravity, so it scales as ``mass * leg length``; kd keeps the damping
    ratio against the leg inertia (``mass * length^2``), so it scales as ``sqrt(kp * inertia)``.
    Aliengo -> Go2 gives 0.516 and 0.476.  The result still has to be checked on the robot.
    """
    kp_ratio = mass_ratio * length_ratio
    kd_ratio = float(np.sqrt(kp_ratio * mass_ratio * length_ratio ** 2))
    return kp_ratio, kd_ratio


def _joint_names():
    """The twelve joint names, in the leg order the policies were trained in."""
    return [f"{leg}_{joint}_joint" for leg in _LEGS for joint in ("haa", "hfe", "kfe")]


def _ee_frames():
    """The foot frames, in the same leg order as :func:`_joint_names`."""
    return [f"{leg}_foot" for leg in _LEGS]


def _unitree_body_map():
    """Gazebo body name -> the same body in a Unitree training URDF (shared by Aliengo, Go1, Go2).

    Gazebo calls the floating base 'base_link' where the URDF calls it 'trunk', and the legs are
    lf/lh/rf/rh + hip/upperleg/lowerleg against FL/RL/FR/RR + hip/thigh/calf.
    """
    return dict(
        {"base_link": "trunk"},
        **{f"{a}_{s}": f"{b}_{t}"
           for a, b in zip(_LEGS, ("FL", "RL", "FR", "RR"))
           for s, t in (("hip", "hip"), ("upperleg", "thigh"), ("lowerleg", "calf"))})


DEFAULTS = {
    # =============================================================================================
    # robot description
    # =============================================================================================
    "joint_names": None,     # required
    "ee_frames": None,       # required, in the same leg order as joint_names
    "ip": None,              # pinged before a --real run
    # NIC of the control PC the robot is wired to, passed to the hardware interface for drivers that
    # bind to a named interface (unitree_sdk2 DDS on the Go2).  None keeps the driver's default.
    # Depends on the machine, not the robot; --network-interface overrides it.
    "network_interface": None,
    "real_robot": False,     # set from the command line, not here
    "spawn_x": 0.0,
    "spawn_y": 0.0,
    "spawn_z": 0.4,          # corrected automatically, see fix_spawn_height
    "spawn_R": 0.0,
    "spawn_P": 0.0,
    "spawn_Y": 0.0,
    # Resting posture the controller folds into before calibrating and standing up.  Keep every
    # joint well clear of its limit: a knee parked against the hard stop is held there by the motor,
    # which shows up as a strained, buzzing robot rather than a resting one.
    "q_fold": None,          # required
    # Total mass in kg from the description.  Used to size the default push and by gait_report.py
    # to normalise foot forces; not used by the control loop.
    "body_mass": None,       # required
    # Present only because BaseController's own initVars would size its log arrays from it.  This
    # controller overrides initVars and logs over ROS topics instead, so nothing here allocates.
    "buffer_size": 1,

    # =============================================================================================
    # loop rate
    # =============================================================================================
    # Control-loop period: 0.002 s = 500 Hz, giving a decimation of 10 against the policy's 50 Hz.
    # It has to be an exact integer multiple of the policy period (20 ms); the controller checks both
    # rates at start-up and refuses to run otherwise, rather than silently sampling the joint
    # velocities, the IMU history and the action history on a different time base than training.
    "control_dt": 0.002,
    "dt": 0.002,             # kept equal to control_dt for BaseController compatibility

    # =============================================================================================
    # gains - per robot, and required
    # =============================================================================================
    # Gains depend on the robot's mass and inertia, so there are no defaults: each robot must set
    # its own.  _scaled_gains() gives a starting point from an already tuned robot.
    #
    # Repositioning the legs on the ground during INIT: softer and better damped than the stand-up
    # set, because the feet are loaded and being dragged, and stiff position control there is what
    # makes the robot judder.
    "kp_fold": None,         # required
    "kd_fold": None,         # required
    # Lifting and holding the body.
    "kp_stand": None,        # required
    "kd_stand": None,        # required
    # Emergency collapse: kp is zero, so joint torque is -kd*qd and the robot sinks under its own
    # weight with the energy bled off instead of dropping.
    "kd_damping": None,      # required
    # While the policy drives.  None takes the values the policy was trained with, read from the
    # policy's json (Isaac DelayedPDActuatorCfg: stiffness 25, damping 0.5).  Overriding these is a
    # deliberate departure from training - the soft damping is what the policy expects.
    "kp_rl": None,
    "kd_rl": None,

    # =============================================================================================
    # motion timing, seconds
    # =============================================================================================
    "settle_duration": 1.0,        # hold the posture we inherit, before calibrating and after
                                   # reaching the fold
    "init_fold_duration": 3.0,     # ramp from the start-up posture into q_fold
    # Worth of *continuously still* samples the bias estimate integrates over.  At 500 Hz that is
    # 1500 samples, which brings the per-axis spread down to a few 1e-4 m/s^2.  The total wait is
    # this plus calib_stable_window.
    "calibration_duration": 5.0,
    "stand_up_duration": 2.0,      # total, split over the stand-up waypoints
    "stand_down_duration": 2.0,
    "damping_ramp_duration": 0.6,  # feed-forward torque ramp-out
    "damping_settle_duration": 2.5,

    # =============================================================================================
    # postures
    # =============================================================================================
    # Stand-up target.  None takes the policy's training default posture, which is the offset its
    # actions are applied to and therefore the smoothest place to hand over.
    "q_stand": None,
    # Tuck the feet under the body before extending; this is what makes a stand-up work from a
    # sprawled pose rather than only from a neat fold.
    "stand_up_via_fold": True,

    # =============================================================================================
    # IMU bias calibration
    # =============================================================================================
    # The window must be *continuously* still: stillness has to hold for calib_stable_window before
    # a single sample is taken, and any motion afterwards throws the window away and starts over.  A
    # bias that absorbed part of a settling transient is worse than no bias at all, because
    # everything downstream then trusts it.
    "calib_stable_window": 1.0,         # s of uninterrupted stillness before sampling starts
    # A settled robot measures about 0.003 rad/s of base rate and 0.012 rad/s of joint rate, so
    # these leave a wide margin while still rejecting a robot that is drifting or being nudged.
    "calib_gyro_threshold": 0.2,       # rad/s
    "calib_joint_vel_threshold": 0.5,  # rad/s
    "calib_wait_timeout": 30.0,         # s before giving up on ever being still
    "calib_max_bias": 2.0,              # m/s^2, a larger estimate is rejected outright
    "calib_max_spread": 0.25,           # m/s^2, per-axis sample std above which the window is redone
    "calib_max_residual": 0.5,          # m/s^2, tolerated |acc - bias| - |g| after calibration
    # Skip the whole procedure and leave the bias at zero.  The bias then shows up in the
    # accelerometer history the policy consumes, so this is for bring-up and for a robot whose bias
    # is already known to be small - not for a run you intend to trust.  Also exposed as
    # --skip-calibration.
    "skip_calibration": False,

    # =============================================================================================
    # safety envelope while standing or walking
    # =============================================================================================
    # Per robot and required, like the gains.
    "max_roll_pitch": None,         # rad, beyond this the robot is on its way over -> DAMPING
    # The same limit during the safe stop.  Keep it at least as wide: that policy is trained on
    # larger reset attitudes and pushes than the walking one.
    "safe_max_roll_pitch": None,    # required
    "max_joint_vel": None,          # rad/s -> DAMPING; required
    "clip_to_joint_limits": None,   # None: on for the real robot, off in simulation

    # =============================================================================================
    # disturbances, simulation only
    # =============================================================================================
    # 'p' shoves the base once in a random horizontal direction; 'P' fires a burst of them; -/+
    # change the magnitude live.  Applied as a wrench through /gazebo/apply_body_wrench, off the
    # control thread, so a push never costs the loop a service round-trip.
    #
    # None gives a training-strength push for the robot's mass: training uses
    # mdp.push_by_setting_velocity over +-PUSH_TRAINING_VELOCITY, i.e. mass * velocity / push_duration
    # (125 N on the Aliengo, 75 N on the Go2).  A larger force tests beyond what the policy has seen.
    "push_force": None,             # N; None = body_mass * PUSH_TRAINING_VELOCITY / push_duration
    "push_duration": 0.2,           # s the wrench is held
    "push_force_step": None,        # N per press of -/+; None = a fifth of push_force
    "push_vertical": False,         # include a random z component; training only ever pushed in xy
    "push_burst_count": 5,          # pushes fired by 'P'
    "push_burst_interval": (1.0, 3.0),   # s, uniform gap between them
    # Gazebo body to push.  None means '<robot>::base_link', which is what the floating base is
    # called once spawned - not 'trunk', which is the URDF's name for it.
    "push_body": None,

    # =============================================================================================
    # operator input
    # =============================================================================================
    "max_lin_vel_cmd": 0.5,   # m/s, the range the policy was trained on
    "max_ang_vel_cmd": 0.5,   # rad/s
    "key_lin_step": 0.1,      # m/s per key press
    "key_ang_step": 0.1,      # rad/s per key press
    "key_speed_presets": (0.0, 0.1, 0.2, 0.3, 0.4),  # forward m/s, keys 1..5
    "joy_dead_zone": 0.08,

    # =============================================================================================
    # telemetry - published on ROS topics, nothing is accumulated in memory
    # =============================================================================================
    # Everything the controller knows goes out as topics so it can be recorded with rosbag and read
    # live in PlotJuggler.  Rates are decimated from the control loop; messages are preallocated and
    # reused, so a tick costs a serialise and no allocation.
    "publish_telemetry": True,
    "telemetry_rate": 100.0,   # Hz, for the per-tick signals (IMU terms, attitude)
    "stats_rate": 10.0,        # Hz, for loop timing and memory
    "topic_namespace": "rl",   # topics land under /<robot>/<topic_namespace>/...
    # Publish the base linear velocity the policy regresses internally.  It is the actor's own
    # estimate, evaluated from weights read out of the policy file, and it is the single most useful
    # signal for judging whether the policy is reading the robot correctly.
    "publish_estimated_velocity": True,
    # Evaluate the actor's state-estimation head at all.  It is a diagnostic, not part of the
    # control path: the policy backbone already consumes the estimate inside the ONNX graph, and
    # the controller re-evaluates the head from weights read out of the same file purely so the
    # number can be seen from outside.  False never reads it and never runs it - one three-layer
    # forward pass less per inference tick - and makes 'publish_estimated_velocity' moot, since
    # there is then nothing to publish.  Also on the command line as --no-estimator.
    "estimate_base_velocity": True,
    # Time every inference tick and publish policy/infer_ms and policy/infer_over_budget.  This is
    # instrumentation, not control.  False drops the two perf_counter calls per inference, both
    # topics, the start-up benchmark and the shutdown timing summary; the benchmark itself still
    # works if called directly.  Worth turning off once the timing question is settled - it is
    # answered in the README - and the loop wants nothing it does not need.  Also on the command
    # line as --no-inference-timing.
    "measure_inference_timing": False,

    # =============================================================================================
    # real-time behaviour
    # =============================================================================================
    # None: on for the real robot, off in simulation.
    "realtime": None,
    "sched_fifo_priority": 80,
    # Pin the control thread to one core.  Only worth it once SCHED_FIFO is actually granted, and
    # best paired with isolcpus so nothing else runs there - see the launcher script.
    "pin_cpu": False,
    "pin_cpu_index": None,     # None: the highest-numbered available core
    # Garbage collection.  The startup heap is frozen out of every later collection, generations 0
    # and 1 keep running (bounded and cheap), and only the full generation-2 scan is deferred to
    # resting states.  Switching the collector off entirely, or deferring generation 1 as well,
    # would be simpler but lets garbage accumulate for the whole of a long walk - exactly what must
    # not happen on a robot.
    "gc_managed": True,
    # Generation 0 and 1 stay enabled: both are bounded and cheap, and generation 1 is what stops
    # anything that survives one gen0 pass from accumulating for the whole run.  Only generation 2 -
    # the full-heap scan, the one that can take milliseconds - is deferred to resting states.
    "gc_gen0_threshold": 2000,
    "gc_gen1_threshold": 10,
    # Warn once if resident memory grows by more than this from the post-startup baseline.  A
    # correctly behaving loop allocates nothing steady-state, so any real growth is a leak.
    "memory_growth_warn_mb": 64.0,
    # How often resident memory is read from procfs.  A file open inside the control loop is an
    # occasional multi-millisecond outlier, so this is deliberately slow - a leak shows up over
    # minutes, not milliseconds.
    "memory_check_rate": 1.0,   # Hz

    # =============================================================================================
    # simulation start-up
    # =============================================================================================
    # Correct spawn_z before the launch runs, so the robot neither drops onto the ground nor starts
    # with its feet in it.  The height is computed for the posture the description's go0 applies.
    "fix_spawn_height": True,
    # Where go0 reads its start-up posture from:
    #   'srdf'  the SRDF 'home' group state, ignoring go0_conf (aliengo_description)
    #   'yaml'  '<go0_conf>' from config/ros_impedance_controller.yaml (go2_description)
    # None detects it from the description's go0 script; set it only to override the detection.
    "spawn_posture_source": None,       # None = detect | 'srdf' | 'yaml'
    # Posture go0 applies at t=0.  With a yaml go0 it is passed to the launch as go0_conf; with an
    # SRDF go0 only 'home' is possible.  Pick a posture with the feet on the ground.
    "spawn_posture_name": "home",
    "spawn_clearance": 0.002,   # m of air left under the feet
    # Radius of the foot collision sphere; the foot frame sits at its centre, so the base has to
    # start this much higher than the pure kinematic foot-to-base distance.
    "foot_radius": 0.0265,
    # rl_flat.world is fast.world with an explicit ODE <constraints> block. Gazebo's default
    # contact_surface_layer lets a foot sink 1 mm before any contact force builds, which a trotting
    # robot feels as a spongy floor: base roll rms while walking goes 5.2 -> 3.1 deg at 0.3 m/s and
    # 5.4 -> 2.0 deg at 0.5 m/s with it at zero, and back to 5.6 / 4.2 deg when it is restored.
    # A reference simulator running the same network on the training URDF sits at 1.5 deg, so
    # this closes most of the gap to Isaac but not all of it.
    "world_name": "rl_flat.world",
    # Start rviz with the simulator.  On by default: it is the intended way to look at the robot,
    # and unlike the Gazebo GUI it renders from the published TF and markers rather than driving
    # the physics window.  It is still another process competing for the machine, so --no-rviz is
    # there for timing work and for a real-robot run on a loaded control PC.
    "use_rviz": True,
    # Broadcast the world -> base_link transform.  rviz is launched with 'world' as its fixed
    # frame, and robot_state_publisher only supplies base_link downwards, so without this there is
    # no path from the fixed frame to the robot and rviz shows nothing.  The base class has its own
    # broadcast inside a kinematics update this controller does not run, so it is done here, at the
    # telemetry rate rather than the control rate: rviz has no use for 500 Hz TF and the traffic is
    # not free.
    "publish_tf": True,
    # rviz config file passed to the launch; None keeps the launch file's own default
    # (ros_impedance_controller/config/operator_floating.rviz).
    "rviz_conf": None,
    # Policy basename under rl_quadruped/policies/, without the '_velocity' suffix.
    # None means "use the robot name".
    "policy_name": None,
    # =============================================================================================
    # the model the policy was trained on
    # =============================================================================================
    # A position-control policy is a feedback law tuned against particular link inertias: its joint
    # target becomes torque through a fixed PD, so each leg's closed-loop response is set by kp/kd
    # against that leg's inertia.  Deployed on a robot whose legs differ by a factor, the response
    # the policy learned to expect is not the response it gets - and standing still, which is held
    # by feedback, degenerates into a residual limit cycle.  So the training model is part of the
    # policy's contract and the controller checks it.
    #
    # 'warn' (the default) reports the differing bodies at start-up; 'off' skips the check.  It
    # only ever reports: aliengo_description now carries Unitree's own values, so a mismatch here
    # means the description has drifted - a stale install space, an edit, or a policy retrained
    # against a different asset - and that is where it should be fixed.  There was briefly a
    # 'match' mode that wrote the trained values into Gazebo at start-up; it is gone, because a
    # controller that silently rewrites the physics engine leaves every other tool on the robot
    # looking at a different machine.
    #
    # The check covers mass, centre of mass and inertia.  It does *not* cover joint friction, which
    # is part of the same contract and was the other half of the mismatch: Isaac draws it uniformly
    # from (0, 0.5) N.m at every reset while aliengo_description had it at exactly zero, and a
    # frictionless robot over-rotates - 0.67 rad/s against a 0.5 rad/s yaw command, which Isaac
    # reproduces at 0.64 when its own friction is switched off.  const.xacro now carries 0.25 N.m,
    # the middle of the trained range, and that is where it belongs; there is nothing to check here
    # because Gazebo does not report joint friction back.
    "training_model_check": "warn",     # 'warn' | 'off'
    # Relative inertia difference below which two bodies count as the same.
    "training_model_tol": 0.02,
    # Gazebo body name -> the link of the same body in the training URDF.  Per-robot, because it
    # is purely a naming-convention map; an empty map disables the check.
    "training_body_map": {},
    # Hand over to the safe-stop policy automatically once the operator command has been at zero
    # this long.  0.0 disables it.  Leaving again is always deliberate - resume walking, stand down,
    # or collapse - because every entry into a policy zeroes the stored command, so a hand-back
    # driven by that command would discard the command that triggered it.
    #
    # Why this exists: the walking policy has two attractors at zero command.  Measured over seven
    # identical Gazebo runs of "walk at 0.3 m/s, then command zero", it settled to ~1e-4 rad/s of
    # base rate in four, and in two it locked into a non-decaying 3.5-3.8 Hz trot in place at
    # 0.04-0.15 rad/s - growing, in one case, from 0.09 to 0.18 rad/s over ten seconds.  Which
    # attractor it lands in is not driven by loop jitter (uncorrelated, r = -0.28..-0.04 across
    # those runs) and not by the model mismatch below (the failures occur with either model): it
    # depends on the gait state at the instant the command drops.  The safe-stop policy, trained
    # only ever at zero command, settled in every run measured.  So if you need "stop" to mean
    # stopped, this is the switch - at the cost of a state change the operator did not ask for.
    "auto_safe_stop_after": 0.0,        # s of continuous zero command; 0.0 = off
    "auto_safe_stop_deadband": 0.02,    # m/s and rad/s below which a command counts as zero
    # Which entry of the contract's "onnx_variants" the safe stop runs.  It has to share the
    # observation and action spaces with the walking variant - the controller checks - because the
    # two hand over mid-stride on a single continuous observation history.
    "safe_policy_variant": "safe",
}


# Besides an entry here, each robot needs a <robot>_description package, a policy trio under
# policies/ (<policy_name>_velocity{.json,_normal.onnx,_safe.onnx}) and, for --real, a
# <robot>_hardware_interface package.  See "Adding a robot" in the README.
ROBOTS = {
    # =============================================================================================
    # Aliengo - 24.94 kg, 0.25 m links
    # =============================================================================================
    "aliengo": {
        # --- description ---------------------------------------------------------------------
        "joint_names": _joint_names(),
        "ee_frames": _ee_frames(),
        "ip": "192.168.123.220",
        "body_mass": 24.937,
        "foot_radius": 0.0265,
        "training_body_map": _unitree_body_map(),

        # --- start-up ------------------------------------------------------------------------
        # aliengo_description's go0 always applies the SRDF 'home': (0.8, 1.7, -2.0), feet on the
        # ground.
        "spawn_posture_name": "home",
        # Knee at -2.40 rather than the -2.70 of params.py's q_fold: the joint limit is -2.775, so
        # -2.70 leaves only 0.075 rad of margin and the knees end up parked against the stop,
        # carrying about 7.5 Nm to stay there.  -2.40 leaves 0.375 rad and rests quietly.
        "q_fold": _legs(0.2, 1.4, -2.70),

        # --- gains ---------------------------------------------------------------------------
        "kp_fold": _gains(60., 60., 60.),
        "kd_fold": _gains(0.5, 0.5, 0.5),
        "kp_stand": _gains(100., 100., 100.),
        "kd_stand": _gains(1.0, 1.0, 1.0),
        "kd_damping": _gains(1.0, 1.0, 1.0),

        # --- safety envelope -----------------------------------------------------------------
        "max_roll_pitch": 1.5,
        "safe_max_roll_pitch": 1.5,
        # ~3x the fastest joint measured while walking (16.3 rad/s).
        "max_joint_vel": 50.0,

        # No 'real' overrides: the control parameters are identical in simulation and on the
        # robot.  The policy was trained against a model of this machine, so if a gain or a duration
        # has to change between the two, that is a modelling error to fix rather than a difference to
        # paper over - and a controller that behaves differently in simulation is one you cannot
        # test there.  The only things that still differ are not modelling parameters: joint-limit
        # clipping and the real-time tuning, both resolved from --real in the controller, and the
        # simulation-only spawn-height correction.
    },

    # =============================================================================================
    # Go2 - 15.10 kg, 0.213 m links
    # =============================================================================================
    # Note: go2_description/urdfs/const.xacro has joint friction at 0.0, while training samples it
    # from (0, 0.5) Nm.  A frictionless robot over-rotates on yaw commands in Gazebo.
    "go2": {
        # --- description ---------------------------------------------------------------------
        "joint_names": _joint_names(),
        "ee_frames": _ee_frames(),
        "ip": "192.168.123.161",
        # NIC of the current control PC (192.168.123.222/24).  The driver and launch file defaults
        # point at other machines, so it is set explicitly here.
        "network_interface": "enp11s0",
        "body_mass": 15.098,
        "foot_radius": 0.02,     # go2_description/urdfs/const.xacro
        "training_body_map": _unitree_body_map(),

        # --- start-up ------------------------------------------------------------------------
        # go2_description's go0 honours go0_conf.  'standDown' is the same sprawled posture as the
        # Aliengo's 'home', (0.8, 1.7, -2.0); the yaml's own 'home' is already standing.
        "spawn_posture_name": "standDown",
        # Knee limit is -2.7227; -2.40 keeps the knees off the hard stop.
        "q_fold": _legs(0.2, 1.6, -2.40),

        # --- gains ---------------------------------------------------------------------------
        # kp from the Aliengo's via _scaled_gains (~0.5x), which gives the same standing sag
        # (~39 mm against the Aliengo's 36 mm).  kd is not scaled down: on hardware an under-damped
        # joint chatters, so it keeps the 1.0 the robot has been run with.
        "kp_fold": _gains(30., 30., 30.),
        "kd_fold": _gains(1.0, 1.0, 1.0),
        "kp_stand": _gains(50., 50., 50.),
        "kd_stand": _gains(1.0, 1.0, 1.0),
        "kd_damping": _gains(1.0, 1.0, 1.0),

        # --- safety envelope -----------------------------------------------------------------
        "max_roll_pitch": 1.5,
        "safe_max_roll_pitch": 1.5,
        # ~3x the fastest joint measured while walking (20.3 rad/s).
        "max_joint_vel": 60.0,
    },
}


# Fields every robot must set itself; gains and safety limits have no sensible shared default.
REQUIRED = ("joint_names", "ee_frames", "q_fold", "body_mass",
            "kp_fold", "kd_fold", "kp_stand", "kd_stand", "kd_damping",
            "max_roll_pitch", "safe_max_roll_pitch", "max_joint_vel")

# Range of the x/y velocity kick applied by mdp.push_by_setting_velocity in training.
PUSH_TRAINING_VELOCITY = 1.0   # m/s


def known_robots():
    """Names of the configured robots."""
    return sorted(ROBOTS)


def get_config(robot_name, real_robot=False):
    """Merged configuration for ``robot_name``, also published as :data:`robot_params`.

    Raises:
        KeyError: if the robot has no entry, if an unknown key is present, or if a required field is
            missing.  Running an untuned machine on silent defaults is worse than refusing to start.
    """
    if robot_name not in ROBOTS:
        raise KeyError(
            f"No RL controller configuration for '{robot_name}'. Add an entry to ROBOTS in "
            f"{__file__}; known robots: {sorted(ROBOTS)}")

    cfg = dict(DEFAULTS)
    robot_cfg = dict(ROBOTS[robot_name])
    real_cfg = robot_cfg.pop("real", {})

    unknown = (set(robot_cfg) | set(real_cfg)) - set(DEFAULTS)
    if unknown:
        raise KeyError(f"Unknown RL controller config keys for '{robot_name}': {sorted(unknown)}")

    cfg.update(robot_cfg)
    if real_robot:
        cfg.update(real_cfg)
    cfg["real_robot"] = real_robot
    cfg["dt"] = cfg["control_dt"]

    missing = [key for key in REQUIRED if cfg.get(key) is None]
    if missing:
        raise KeyError(f"RL controller config for '{robot_name}' is missing: {missing}")

    # Default push: the same velocity change as in training, whatever the robot's mass.
    if cfg["push_force"] is None:
        cfg["push_force"] = cfg["body_mass"] * PUSH_TRAINING_VELOCITY / cfg["push_duration"]
    if cfg["push_force_step"] is None:
        cfg["push_force_step"] = cfg["push_force"] / 5.0

    # BaseController resolves everything through conf.robot_params[robot_name]; handing it this
    # dictionary is what keeps params.py out of the picture entirely.
    robot_params[robot_name] = cfg
    return cfg
