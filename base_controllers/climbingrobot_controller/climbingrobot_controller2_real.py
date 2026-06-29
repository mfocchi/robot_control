# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

import rospy as ros
from base_controllers.utils.math_tools import *
import pinocchio as pin

np.set_printoptions(threshold=np.inf, precision=5, linewidth=1000, suppress=True)
import matplotlib.pyplot as plt
from numpy import nan
from base_controllers.utils.common_functions import plotJoint, plotFrameLinear, spawnMesh
from termcolor import colored
import os
import tf
from base_controllers.base_controller_fixed import BaseControllerFixed
from geometry_msgs.msg import Wrench, Point
from gazebo_msgs.msg import ContactsState
from std_srvs.srv import Trigger
import scipy.io.matlab as mio
import rospkg
from base_controllers.utils.matlab_conversions import mat_vector2python, mat_matrix2python
import matlab.engine
from base_controllers.utils.rosbag_recorder import RosbagControlledRecorder
import sys

np.set_printoptions(threshold=np.inf, precision=5, linewidth=10000, suppress=True)
from base_controllers.utils.common_functions import checkRosMaster
import base_controllers.params as conf

robotName = "climbingrobot2"
# real robot msgs
import std_msgs, geometry_msgs
from climbingrobot_hardware_interface.msg import RopeCommand
from climbingrobot_hardware_interface.msg import PropellerCommand
from climbingrobot_hardware_interface.msg import RopeTelemetry
from climbingrobot_hardware_interface.msg import AlpineBodyTelemetry
# real robot services
from climbingrobot_hardware_interface.srv import AlpineBodyCommand, AlpineBodyCommandRequest
from climbingrobot_hardware_interface.srv import RopeControlMode, RopeControlModeRequest
from base_controllers.utils.common_functions import startNode, checkRosMaster, launchFileNode
from base_controllers.utils.math_tools import quaternion_matrix
from base_controllers.utils.ros_publish import RosPub
from orientation_controller import OrientationController
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

# Find the ROS package path
rospack = rospkg.RosPack()
climbing_hw_path = rospack.get_path("climbingrobot_hardware_interface")
# Add climbing_hardware_interface/scripts to Python import path
install_prefix = os.path.abspath(os.path.join(climbing_hw_path, "..", ".."))
scripts_path = os.path.join(install_prefix, "lib", "climbingrobot_hardware_interface")
sys.path.insert(0, scripts_path)
from homing_procedure import WinchStartupSequence


class ClimbingrobotController(BaseControllerFixed):
    def __init__(self, robot_name="ur5"):
        self.EXTERNAL_FORCE = False
        self.landing = True  # do landing
        self.MPC_control = True
        self.PLOT_MPC = False

        self.SAVE_BAG = False  # does not show rope vectors
        self.rope_index = np.array([2, 8])  # 'wire_base_prismatic_r', 'wire_base_prismatic_l',
        self.leg_index = np.array([12, 13, 14])
        self.wheel_index = np.array([16, 18])  # 'wheel_joint_l',  'wheel_joint_r'
        self.hip_pitch_joint = 12
        self.hip_roll_joint = 13
        self.base_passive_joints = np.array([3, 4, 5, 9, 10, 11])
        self.anchor_passive_joints = np.array([0, 1, 6, 7])
        self.OBSTACLE_AVOIDANCE = 'mesh'  # 'none', 'mesh'

        if self.MPC_control:
            sys.path.insert(0, './codegen_mpc')

        if self.OBSTACLE_AVOIDANCE == 'mesh':
            sys.path.insert(0, './codegen_mesh_normal')
            from base_controllers.components.terrain_manager import TerrainManager
            # generate terrain
            # Parameters (direct translation from MATLAB)
            wall_depth = 1  # how
            grid_size = 100
            max_ridge_depth = 0.5
            seed = "default"
            Lz = -20  # Height of wall in meters
            Ly = 5  # Width (horizontal extent) of wall in meters
            # Generate rock wall map
            self.terrainManager = TerrainManager()
            self.mesh_x, self.mesh_y, self.mesh_z = self.terrainManager.generate_rock_wall_map(Lz, Ly, grid_size,
                                                                                               wall_depth,
                                                                                               max_ridge_depth, seed,
                                                                                               x_offset=-0.5)
        else:
            sys.path.insert(0, './codegen')

        self.force_scale = 60.
        self.mountain_thickness = 0.1  # TODO call the launch file passing this parameter
        self.r_leg = 0.3
        self.real_robot = conf.robot_params[robot_name]['real_robot']
        super().__init__(robot_name=robot_name)
        print("Initialized climbingrobot controller---------------------------------------------------------------")

    def send_alpine_raw(self, cmd: str):
        msg = std_msgs.msg.String()
        msg.data = cmd
        self.pub_alpine_cmdraw.publish(msg)

    def apply_propeller_command(self, prop_thrusts=[0.,0.,0.,0.]):
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[0],
                               self.orientControl.b_propeller_axes[0] * prop_thrusts[0]/self.force_scale , "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[1],
                               self.orientControl.b_propeller_axes[1] * prop_thrusts[1] / self.force_scale, "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[2],
                               self.orientControl.b_propeller_axes[2] * prop_thrusts[2] / self.force_scale, "blue", scale=1.5)
        self.ros_pub.add_arrow(self.base_pos + self.w_R_b @ self.orientControl.b_propeller_pos[3],
                               self.orientControl.b_propeller_axes[3] * prop_thrusts[3] / self.force_scale, "blue", scale=1.5)
        msg =  PropellerCommand()
        msg.propeller_thrust_0 = prop_thrusts[0]
        msg.propeller_thrust_1 = prop_thrusts[1]
        msg.propeller_thrust_2 = prop_thrusts[2]
        msg.propeller_thrust_3 = prop_thrusts[3]
        self.pub_propeller_command.publish(msg)

    def send_alpine_wrench(self, fx=0.0, fy=0.0, mz=0.0):
        msg = Wrench()
        msg.force.x = float(fx)
        msg.force.y = float(fy)
        msg.force.z = 0.0
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = float(mz)
        self.pub_alpine_wrench.publish(msg)

    def enable_attitude_hold(self):
        self.send_alpine_raw("attzero")
        ros.sleep(0.05)
        self.send_alpine_raw("atton")
        self.send_alpine_wrench(0.0, 0.0, 0.0)

    def disable_attitude_hold(self):
        self.send_alpine_raw("attoff")
        self.send_alpine_wrench(0.0, 0.0, 0.0)

    def getRobotMass(self):
        total_robot_mass = 5.2  # todo hardcode this
        return total_robot_mass

    def get_alpine_param(self, name, default=None):
        """Read ALPINE params with safe fallbacks.

        Priority:
          1. private param of this controller node: ~name
          2. global shared param from low-level launch: /alpine/name
          3. robot_params[robot_name][name]
          4. default
        """
        robot_conf = conf.robot_params.get(self.robot_name, {})
        fallback = robot_conf.get(name, default)

        try:
            if ros.has_param("~" + name):
                return ros.get_param("~" + name)
            shared_name = "/alpine/" + name
            if ros.has_param(shared_name):
                return ros.get_param(shared_name)
            global_name = "/" + name
            if ros.has_param(global_name):
                return ros.get_param(global_name)
        except Exception:
            pass

        return fallback

    def initRopeOffsets(self):
        # Offsets are physical rope lengths at the end of homing/rope_zero.
        # Runtime conversion convention:
        #   telemetry:  L_abs = home_offset + rope_sign * L_raw
        #   command:    L_raw_ref = rope_sign * (L_abs_ref - home_offset)
        self.left_home_offset_m = float(self.get_alpine_param('left_home_offset_m', 0.0))
        self.right_home_offset_m = float(self.get_alpine_param('right_home_offset_m', 0.53))
        self.left_rope_sign = float(self.get_alpine_param('left_rope_sign', 1.0))
        self.right_rope_sign = float(self.get_alpine_param('right_rope_sign', -1.0))
        self.left_rope_axis = str(self.get_alpine_param('left_rope_axis', '-x'))
        self.homing_test_delta_m = float(self.get_alpine_param('homing_test_delta_m', 0.5))

        # Pipeline reale: prima porta il robot giù in position mode, poi abilita il salto.
        # Il valore positivo allunga entrambe le corde, quindi il corpo scende lungo la parete.
        self.pipeline_position_then_jump_enabled = bool(
            self.get_alpine_param('pipeline_position_then_jump_enabled', True)
        )
        # 'manual' calls /alpine/jump after the position move.
        # 'optimized' enters the existing optimizer/stateMachineLoop path.
        self.pipeline_jump_mode = str(self.get_alpine_param('pipeline_jump_mode', 'manual')).strip().lower()
        # Touchdown detection is currently not implemented in this controller.
        # Keep it disabled by default for the new pipeline unless explicitly requested.
        self.pipeline_landing_enabled = bool(self.get_alpine_param('pipeline_landing_enabled', False))
        self.prejump_drop_m = float(self.get_alpine_param('prejump_drop_m', 1.20))
        self.prejump_position_tolerance_m = float(self.get_alpine_param('prejump_position_tolerance_m', 0.03))
        self.prejump_position_timeout_s = float(self.get_alpine_param('prejump_position_timeout_s', 20.0))
        self.prejump_position_min_wait_s = float(self.get_alpine_param('prejump_position_min_wait_s', 0.5))
        self.prejump_settle_s = float(self.get_alpine_param('prejump_settle_s', 0.5))
        self.prejump_abort_on_timeout = bool(self.get_alpine_param('prejump_abort_on_timeout', True))

        # Optional lateral sine test between the pre-jump position move and the jump.
        # This is finite by design; otherwise the pipeline would never reach /alpine/jump.
        self.prejump_lateral_sine_enabled = bool(
            self.get_alpine_param('prejump_lateral_sine_enabled', True)
        )
        self.prejump_lateral_sine_amp_fy = float(
            self.get_alpine_param('prejump_lateral_sine_amp_fy', 0.60)
        )
        self.prejump_lateral_sine_freq_hz = float(
            self.get_alpine_param('prejump_lateral_sine_freq_hz', 1.0)
        )
        self.prejump_lateral_sine_duration_s = float(
            self.get_alpine_param('prejump_lateral_sine_duration_s', 2.0)
        )
        self.prejump_lateral_sine_ramp_s = float(
            self.get_alpine_param('prejump_lateral_sine_ramp_s', 0.25)
        )

        # Extra safety dwell times around mode transitions.
        # These keep the winches in position hold before switching ownership to
        # jump.py, reducing rope-force discontinuities.
        self.post_homing_position_hold_s = float(
            self.get_alpine_param('post_homing_position_hold_s', 0.50)
        )
        self.prejump_position_settle_after_reached_s = float(
            self.get_alpine_param('prejump_position_settle_after_reached_s', 0.50)
        )

        self.rviz_use_measured_rope_joints = bool(self.get_alpine_param('rviz_use_measured_rope_joints', True))
        self.rviz_show_rope_length_text = bool(self.get_alpine_param('rviz_show_rope_length_text', True))

        # Pure RViz/Locosim startup home pose, kept identical to the old real
        # controller.  These values are only used to upload robot_description and
        # seed /joint_states at startup; the physical odometry geometry below is
        # still used after homing for the offset calculation.
        self.rviz_home_anchor_left_xyz = np.array(
            self.get_alpine_param('rviz_home_anchor_left_xyz', [0.2, 0.0, 2.40]), dtype=float
        )
        self.rviz_home_anchor_right_xyz = np.array(
            self.get_alpine_param('rviz_home_anchor_right_xyz', [0.2, 2.2, 2.40]), dtype=float
        )
        self.rviz_home_base_width = float(self.get_alpine_param('rviz_home_base_width', 0.4))
        self.rviz_home_publish_s = float(self.get_alpine_param('rviz_home_publish_s', 0.5))
        # Backward-compatible alias used by older local edits of this file.
        self.rviz_homing_pose_publish_s = self.rviz_home_publish_s

        self.anchor_left_xyz = np.array(self.get_alpine_param('anchor_left_xyz', [0.45, 0.0, 2.50]), dtype=float)
        self.anchor_right_xyz = np.array(self.get_alpine_param('anchor_right_xyz', [0.45, 0.65, 2.50]), dtype=float)
        self.anchor_distance_y = float(self.anchor_right_xyz[1] - self.anchor_left_xyz[1])
        self.anchor_pos = self.anchor_left_xyz.copy()
        self.anchor_pos2 = self.anchor_right_xyz.copy()
        self.body_origin_from_left_attachment = np.array(
            self.get_alpine_param('body_origin_from_left_attachment_xyz', [0.0, 0.0, -0.55]), dtype=float
        )
        self.right_attachment_from_left_body = np.array(
            self.get_alpine_param('right_attachment_from_left_body_xyz', [0.0, -0.55, 0.0]), dtype=float
        )

        # Physical span used after homing.  Do not overwrite base_width here:
        # base_width is also used by the old Locosim/RViz startup visualization,
        # and at power-on it must stay at the historical homing value set in
        # startRealRobot().
        span = np.linalg.norm(self.right_attachment_from_left_body)
        if np.isfinite(span) and span > 1e-6:
            self.physical_hoist_distance = float(span)
        else:
            self.physical_hoist_distance = float(getattr(self, 'physical_hoist_distance', self.hoist_distance))

        print(colored(
            "ALPINE rope offsets: "
            f"left offset={self.left_home_offset_m:.3f} sign={self.left_rope_sign:+.1f}, "
            f"right offset={self.right_home_offset_m:.3f} sign={self.right_rope_sign:+.1f}, "
            f"axis={self.left_rope_axis}, homing_delta={self.homing_test_delta_m:.3f}, "
            f"rviz_home_L={self.rviz_home_anchor_left_xyz.tolist()}, "
            f"rviz_home_R={self.rviz_home_anchor_right_xyz.tolist()}, "
            f"prejump_drop={self.prejump_drop_m:.3f}, "
            f"pipeline={self.pipeline_position_then_jump_enabled}, "
            f"jump_mode={self.pipeline_jump_mode}, landing={self.pipeline_landing_enabled}, "
            f"lateral_sine={self.prejump_lateral_sine_enabled} "
            f"amp={self.prejump_lateral_sine_amp_fy:.3f} "
            f"freq={self.prejump_lateral_sine_freq_hz:.3f} "
            f"duration={self.prejump_lateral_sine_duration_s:.3f}, "
            f"post_homing_hold={self.post_homing_position_hold_s:.3f}, "
            f"position_settle={self.prejump_position_settle_after_reached_s:.3f}",
            "blue"
        ))

    def rope_sign(self, side: str) -> float:
        if side == "left":
            return self.left_rope_sign
        if side == "right":
            return self.right_rope_sign
        raise ValueError(f"Unknown rope side: {side}")

    def rope_home_offset(self, side: str) -> float:
        if side == "left":
            return self.left_home_offset_m
        if side == "right":
            return self.right_home_offset_m
        raise ValueError(f"Unknown rope side: {side}")

    def rope_abs_length(self, side: str, raw_length: float) -> float:
        return self.rope_home_offset(side) + self.rope_sign(side) * float(raw_length)

    def rope_abs_velocity(self, side: str, raw_velocity: float) -> float:
        return self.rope_sign(side) * float(raw_velocity)

    def rope_raw_reference_from_abs(self, side: str, abs_length_ref: float) -> float:
        return self.rope_sign(side) * (float(abs_length_ref) - self.rope_home_offset(side))

    def scalar_time(self) -> float:
        """Return controller time as a scalar even if legacy code stored it as a 1-element array."""
        try:
            return float(np.asarray(self.time).reshape(-1)[0])
        except Exception:
            return float(self.time)

    def publish_rope_position_abs(self, side: str, abs_length_ref: float, rope_force=0.0, rope_velocity=0.0):
        """Publish a physical absolute rope length reference.

        Call this instead of publish_command(... rope_position=...) whenever the
        desired value is expressed in the odometry/RViz physical convention.
        """
        raw_ref = self.rope_raw_reference_from_abs(side, abs_length_ref)
        raw_vel = self.rope_sign(side) * float(rope_velocity)
        self.homingProcedure.publish_command(
            side,
            rope_force=float(rope_force),
            rope_velocity=raw_vel,
            rope_position=raw_ref,
        )

    def publish_rope_torque_forces(self, left_force=0.0, right_force=0.0):
        """Publish rope-force commands in torque mode.

        The low-level winch nodes ignore NaN position/velocity fields, so this
        message changes only rope_force while the controller is in torque mode.
        """
        if not hasattr(self, 'pub_rope_command_l') or not hasattr(self, 'pub_rope_command_r'):
            return

        now = ros.Time.now()
        nan_value = float('nan')

        left = RopeCommand()
        left.header.stamp = now
        left.rope_force = float(left_force)
        left.rope_velocity = nan_value
        left.rope_position = nan_value

        right = RopeCommand()
        right.header.stamp = now
        right.rope_force = float(right_force)
        right.rope_velocity = nan_value
        right.rope_position = nan_value

        self.pub_rope_command_l.publish(left)
        self.pub_rope_command_r.publish(right)

    def publish_current_rope_position_hold(self):
        """Hold the currently measured absolute rope lengths in position mode."""
        if not hasattr(self, 'homingProcedure'):
            return
        self.publish_rope_position_abs("left", self.rope_left_length)
        self.publish_rope_position_abs("right", self.rope_right_length)

    def publish_rope_length_topics_and_text(self):
        """Publish actual absolute rope lengths for RViz and debugging."""
        # Topics are useful even if RViz text markers are disabled.
        if hasattr(self, 'pub_rope_left_abs_length'):
            msg_l = std_msgs.msg.Float32()
            msg_l.data = float(self.rope_left_length)
            self.pub_rope_left_abs_length.publish(msg_l)

        if hasattr(self, 'pub_rope_right_abs_length'):
            msg_r = std_msgs.msg.Float32()
            msg_r.data = float(self.rope_right_length)
            self.pub_rope_right_abs_length.publish(msg_r)

        if not getattr(self, 'rviz_show_rope_length_text', True):
            return
        if not hasattr(self, 'pub_rope_length_marker'):
            return

        now = ros.Time.now()

        def finite_point(v, fallback):
            vv = np.array(v, dtype=float)
            if vv.shape != (3,) or not np.all(np.isfinite(vv)):
                return np.array(fallback, dtype=float)
            return vv

        left_mid = 0.5 * (
            finite_point(getattr(self, 'anchor_pos', np.zeros(3)), np.zeros(3)) +
            finite_point(getattr(self, 'hoist_l_pos', np.zeros(3)), np.zeros(3))
        )
        right_mid = 0.5 * (
            finite_point(getattr(self, 'anchor_pos2', np.zeros(3)), np.zeros(3)) +
            finite_point(getattr(self, 'hoist_r_pos', np.zeros(3)), np.zeros(3))
        )

        def publish_text(marker_id, text, pos, r, g, b):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = "world"
            m.ns = "actual_rope_lengths"
            m.id = int(marker_id)
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.z = 0.08
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = 1.0
            m.text = text
            m.lifetime = ros.Duration(0.25)
            self.pub_rope_length_marker.publish(m)

        left_text = f"L actual: {self.rope_left_length:.3f} m"
        right_text = f"R actual: {self.rope_right_length:.3f} m"
        if getattr(self, 'rope_offsets_active', False):
            left_text += f"\nraw: {self.rope_left_raw_length:.3f} m"
            right_text += f"\nraw: {self.rope_right_raw_length:.3f} m"

        publish_text(0, left_text, left_mid + np.array([0.0, -0.06, 0.08]), 0.0, 1.0, 0.0)
        publish_text(1, right_text, right_mid + np.array([0.0, 0.06, 0.08]), 0.0, 1.0, 0.0)

    def start_prejump_position_drop(self):
        """Command the pre-jump descent in absolute rope-length convention."""
        self.setRopeControlMode('closed_loop_position')
        ros.sleep(0.05)

        drop = max(0.0, float(getattr(self, 'prejump_drop_m', 1.20)))
        self.prejump_left_ref_abs = float(self.rope_left_length) + drop
        self.prejump_right_ref_abs = float(self.rope_right_length) + drop
        self.prejump_position_start_time = self.scalar_time()
        self.prejump_position_command_sent = True

        self.publish_rope_position_abs("left", self.prejump_left_ref_abs)
        self.publish_rope_position_abs("right", self.prejump_right_ref_abs)

        print(colored(
            "PRE-JUMP POSITION: "
            f"drop={drop:.3f} m, "
            f"left {self.rope_left_length:.3f}->{self.prejump_left_ref_abs:.3f} m, "
            f"right {self.rope_right_length:.3f}->{self.prejump_right_ref_abs:.3f} m",
            "yellow"
        ))

    def monitor_prejump_position_drop(self):
        """Return 'done', 'running', or 'timeout' for the pre-jump position move."""
        if not getattr(self, 'prejump_position_command_sent', False):
            self.start_prejump_position_drop()
            return 'running'

        err_l = abs(float(self.prejump_left_ref_abs) - float(self.rope_left_length))
        err_r = abs(float(self.prejump_right_ref_abs) - float(self.rope_right_length))
        err = max(err_l, err_r)
        elapsed = self.scalar_time() - float(self.prejump_position_start_time)

        if not hasattr(self, 'prejump_position_print_counter'):
            self.prejump_position_print_counter = 0
        if np.mod(self.prejump_position_print_counter, 200) == 0:
            print(colored(
                "PRE-JUMP POSITION monitor: "
                f"L={self.rope_left_length:.3f}/{self.prejump_left_ref_abs:.3f} "
                f"R={self.rope_right_length:.3f}/{self.prejump_right_ref_abs:.3f} "
                f"err={err:.3f} m elapsed={elapsed:.2f} s",
                "yellow"
            ))
        self.prejump_position_print_counter += 1

        if elapsed >= float(self.prejump_position_min_wait_s) and err <= float(self.prejump_position_tolerance_m):
            print(colored(
                f"PRE-JUMP POSITION reached: err={err:.3f} m, elapsed={elapsed:.2f} s",
                "green"
            ))
            return 'done'

        if elapsed >= float(self.prejump_position_timeout_s):
            print(colored(
                f"PRE-JUMP POSITION timeout: err={err:.3f} m, elapsed={elapsed:.2f} s",
                "red"
            ))
            return 'timeout'

        return 'running'

    def publish_prejump_reference_hold(self):
        """Keep the reached pre-jump rope references active in position mode."""
        try:
            left_ref = float(self.prejump_left_ref_abs)
            right_ref = float(self.prejump_right_ref_abs)
            if not np.isfinite(left_ref):
                left_ref = float(self.rope_left_length)
            if not np.isfinite(right_ref):
                right_ref = float(self.rope_right_length)
        except Exception:
            left_ref = float(self.rope_left_length)
            right_ref = float(self.rope_right_length)

        self.publish_rope_position_abs("left", left_ref)
        self.publish_rope_position_abs("right", right_ref)

    def start_prejump_position_settle(self):
        """Hold in position mode briefly after reaching the drop target."""
        self.stateMachine = 'prejump_position_settle'
        self.prejump_position_settle_start_time = self.scalar_time()
        self.prejump_position_settle_print_counter = 0
        self.publish_prejump_reference_hold()

        msg = std_msgs.msg.String()
        msg.data = 'prejump_position_settle_started'
        self.pub_goal_status.publish(msg)

        print(colored(
            "PRE-JUMP POSITION SETTLE started: "
            f"duration={float(getattr(self, 'prejump_position_settle_after_reached_s', 0.50)):.3f} s",
            "yellow"
        ))

    def monitor_prejump_position_settle(self):
        """Return 'done' or 'running' while holding the final position reference."""
        duration = max(0.0, float(getattr(self, 'prejump_position_settle_after_reached_s', 0.50)))
        elapsed = self.scalar_time() - float(getattr(self, 'prejump_position_settle_start_time', self.scalar_time()))

        # Keep refreshing the final position reference at a low rate.  This avoids
        # leaving a stale command if another node briefly touched /winch/*/command.
        if np.mod(getattr(self, 'prejump_position_settle_print_counter', 0), 100) == 0:
            self.publish_prejump_reference_hold()
            print(colored(
                "PRE-JUMP POSITION SETTLE monitor: "
                f"elapsed={elapsed:.2f}/{duration:.2f} s, "
                f"L={self.rope_left_length:.3f}, R={self.rope_right_length:.3f}",
                "yellow"
            ))
        self.prejump_position_settle_print_counter = getattr(self, 'prejump_position_settle_print_counter', 0) + 1

        if duration <= 0.0 or elapsed >= duration:
            self.publish_prejump_reference_hold()
            msg = std_msgs.msg.String()
            msg.data = 'prejump_position_settle_done'
            self.pub_goal_status.publish(msg)
            print(colored(
                f"PRE-JUMP POSITION SETTLE finished after {max(elapsed, 0.0):.2f} s",
                "green"
            ))
            return 'done'

        return 'running'

    def continue_after_prejump_position_settle(self):
        """Advance from the settle phase to sine or jump."""
        if getattr(self, 'prejump_lateral_sine_enabled', True):
            self.start_prejump_lateral_sine()
            return False
        return self.start_jump_after_prejump_sequence()

    def start_prejump_lateral_sine(self):
        """Start the finite lateral sine phase between position and jump."""
        self.prejump_lateral_sine_start_time = self.scalar_time()
        self.prejump_lateral_sine_print_counter = 0
        self.stateMachine = 'prejump_lateral_sine'

        msg = std_msgs.msg.String()
        msg.data = 'prejump_lateral_sine_started'
        self.pub_goal_status.publish(msg)

        print(colored(
            "PRE-JUMP LATERAL SINE started: "
            f"amp_fy={float(getattr(self, 'prejump_lateral_sine_amp_fy', 0.60)):.3f}, "
            f"freq={float(getattr(self, 'prejump_lateral_sine_freq_hz', 1.0)):.3f} Hz, "
            f"duration={float(getattr(self, 'prejump_lateral_sine_duration_s', 2.0)):.3f} s",
            "yellow"
        ))

    def monitor_prejump_lateral_sine(self):
        """Run the lateral sine. Return 'done' or 'running'."""
        duration = max(0.0, float(getattr(self, 'prejump_lateral_sine_duration_s', 2.0)))
        elapsed = self.scalar_time() - float(getattr(self, 'prejump_lateral_sine_start_time', self.scalar_time()))

        if duration <= 0.0 or elapsed >= duration:
            self.send_alpine_wrench(0.0, 0.0, 0.0)
            self.prop_force_y = 0.0
            print(colored(
                f"PRE-JUMP LATERAL SINE finished after {max(elapsed, 0.0):.2f} s",
                "green"
            ))
            msg = std_msgs.msg.String()
            msg.data = 'prejump_lateral_sine_done'
            self.pub_goal_status.publish(msg)
            return 'done'

        amp = float(getattr(self, 'prejump_lateral_sine_amp_fy', 0.60))
        freq = float(getattr(self, 'prejump_lateral_sine_freq_hz', 1.0))
        ramp_s = max(0.0, float(getattr(self, 'prejump_lateral_sine_ramp_s', 0.25)))

        # Smooth start/stop to avoid a sharp lateral wrench step before jump.
        envelope = 1.0
        if ramp_s > 1e-6:
            envelope = min(1.0, max(0.0, elapsed / ramp_s), max(0.0, (duration - elapsed) / ramp_s))

        fy = amp * envelope * np.sin(2.0 * np.pi * freq * elapsed)
        self.send_alpine_wrench(fx=0.0, fy=fy, mz=0.0)
        self.prop_force_y = fy

        if np.mod(getattr(self, 'prejump_lateral_sine_print_counter', 0), 100) == 0:
            print(colored(
                "PRE-JUMP LATERAL SINE monitor: "
                f"elapsed={elapsed:.2f}/{duration:.2f} s, fy={fy:.3f}",
                "yellow"
            ))
        self.prejump_lateral_sine_print_counter = getattr(self, 'prejump_lateral_sine_print_counter', 0) + 1
        return 'running'

    def start_jump_after_prejump_sequence(self):
        """After position and lateral sine, start the selected jump mode."""
        if str(getattr(self, 'pipeline_jump_mode', 'manual')).lower() in ('manual', 'service', 'jump_service'):
            ok = self.trigger_manual_jump_service_once()
            msg = std_msgs.msg.String()
            msg.data = 'manual_jump_started' if ok else 'manual_jump_error'
            self.pub_goal_status.publish(msg)
            self.stateMachine = 'manual_jump_started' if ok else 'position_error'
            return not ok

        self.arm_optimized_jump_after_position()
        return False

    def trigger_manual_jump_service_once(self):
        """Call /alpine/jump once after the pre-jump position move."""
        if getattr(self, 'manual_jump_service_called', False):
            return True
        if not hasattr(self, 'manual_jump_service'):
            print(colored("Manual jump service proxy not available", "red"))
            return False

        try:
            resp = self.manual_jump_service()
            self.manual_jump_service_called = bool(resp.success)
            if resp.success:
                print(colored(f"/alpine/jump started: {resp.message}", "green"))
                return True
            print(colored(f"/alpine/jump rejected: {resp.message}", "red"))
            return False
        except ros.ServiceException as e:
            ros.logerr("/alpine/jump service call failed: %s" % e)
            return False

    def arm_optimized_jump_after_position(self):
        self.startJump = self.scalar_time() + float(getattr(self, 'prejump_settle_s', 0.5))
        self.stateMachine = 'idle'
        print(colored(f"PRE-JUMP POSITION done -> optimized jump armed at t={self.startJump:.2f}", "green"))

    def publish_initial_homing_pose_for_rviz(self, duration_s=None):
        """Seed robot_state_publisher/RViz with the old startup home pose.

        This runs before the winch homing sequence.  It does not affect odometry,
        rope_zero, or any winch command; it only publishes the Locosim q_0 joint
        state long enough for RViz to receive the robot and the arganelli in the
        historical homing location.
        """
        if not hasattr(self, 'pub_joints'):
            return

        if duration_s is None:
            duration_s = getattr(self, 'rviz_home_publish_s', 0.5)
        duration_s = max(float(duration_s), 0.0)

        msg = JointState()
        msg.name = list(self.joint_names)
        q_home = np.array(self.q_des_q0, dtype=float).copy()
        self.q_des = q_home.copy()
        msg.position = q_home.tolist()

        rate = ros.Rate(50)
        t_end = ros.Time.now() + ros.Duration(duration_s)
        first = True
        while first or (not ros.is_shutdown() and ros.Time.now() < t_end):
            first = False
            msg.header.stamp = ros.Time.now()
            self.pub_joints.publish(msg)
            if duration_s <= 0.0:
                break
            rate.sleep()

    def axis_vector_from_rot(self, R: np.ndarray, axis_name: str) -> np.ndarray:
        axis_name = str(axis_name).strip().lower()
        mapping = {
            'x': R[:, 0], '-x': -R[:, 0],
            'y': R[:, 1], '-y': -R[:, 1],
            'z': R[:, 2], '-z': -R[:, 2],
        }
        v = np.array(mapping.get(axis_name, -R[:, 0]), dtype=float)
        n = np.linalg.norm(v)
        if n < 1e-9:
            return np.array([1.0, 0.0, 0.0])
        return v / n

    def estimateRobotVelFromStates(self, l1, l2, psi, l1d, l2d, psid):
        if l1 != 0 and l2 != 0:
            # first estimate position
            px = l1 * np.sin(psi) * np.sqrt(1 - (self.anchor_distance_y ** 2 + l1 ** 2 - l2 ** 2) ** 2 / (
                        4 * self.anchor_distance_y ** 2 * l1 ** 2))
            py = (self.anchor_distance_y ** 2 + l1 ** 2 - l2 ** 2) / (2 * self.anchor_distance_y)
            pz = -l1 * np.cos(psi) * np.sqrt(1 - (self.anchor_distance_y ** 2 + l1 ** 2 - l2 ** 2) ** 2 / (
                        4 * self.anchor_distance_y ** 2 * l1 ** 2))

            # temp vars to simplify equation
            px_l1 = px / l1
            n_pz_l1 = -pz / l1
            px_l1_sinpsi = px / l1 / math.sin(psi + 0.00001)  # to avoid division by zero
            py2b = py * 2 * self.anchor_distance_y

            pdx = l1d * px_l1 + l1 * n_pz_l1 * psid + (py2b * math.sin(psi) * (
                        l1d * pow(self.anchor_distance_y, 2) - l1d * pow(l1, 2) + 2 * l2d * l1 * l2 - l1d * pow(l2,
                                                                                                                2))) / (
                              4 * pow(self.anchor_distance_y, 2) * pow(l1, 2) * px_l1_sinpsi)
            pdy = (l1 * l1d - l2 * l2d) / self.anchor_distance_y
            pdz = l1 * psid * px_l1 - l1d * n_pz_l1 - (py2b * math.cos(psi) * (
                        l1d * pow(self.anchor_distance_y, 2) - l1d * pow(l1, 2) + 2 * l2d * l1 * l2 - l1d * pow(l2,
                                                                                                                2))) / (
                              4 * pow(self.anchor_distance_y, 2) * pow(l1, 2) * px_l1_sinpsi)
            base_vel = np.array([pdx, pdy, pdz])
        else:
            base_vel = np.zeros(3)

        return base_vel

    def updateKinematicsDynamics(self):
        # get measured quantities
        self.w_R_rope = (quaternion_matrix(self.rope_l_imu_orientation))[:3, :3]

        self.w_R_b = quaternion_matrix(self.body_imu_orientation)[:3, :3]
        self.w_omega_b = self.body_imu_angular_velocity  # TODO double check

        # rope gravity terms TO BE RECOMPUTED TODO
        # self.g #

        # Before homing is completed, keep the original Locosim/RViz model
        # path exactly as it was: startup shows the old homing configuration.
        # After homing/rope_zero, switch to the physical offset convention so the
        # model/reference calculations use the measured post-home rope lengths.
        if not getattr(self, 'rope_offsets_active', False):
            self.anchor_pos = np.array(
                [conf.robot_params[self.robot_name]['spawn_x'], conf.robot_params[self.robot_name]['spawn_y'],
                 conf.robot_params[self.robot_name]['spawn_z']])
            self.anchor_pos2 = np.array(
                [conf.robot_params[self.robot_name]['spawn_2x'], conf.robot_params[self.robot_name]['spawn_2y'],
                 conf.robot_params[self.robot_name]['spawn_2z']])
            self.anchor_distance_y = conf.robot_params[self.robot_name]['spawn_2y'] - conf.robot_params[self.robot_name][
                'spawn_y']

            # Original Locosim estimate from rope IMU + home-relative rope length.
            # Use the old visual base width even if physical body geometry is
            # already loaded for the post-homing odometry calculation.
            visual_base_width = float(getattr(self, 'rviz_home_base_width', self.base_width))
            com_offset = self.w_R_b[2] * 0.0  # TODO
            x_rope_l_attach = self.anchor_pos + self.w_R_rope[0] * self.l_1
            self.base_pos = x_rope_l_attach + self.w_R_b[1] * (visual_base_width / 2) + com_offset

            self.hoist_l_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, -visual_base_width / 2, 0.0]))
            self.hoist_r_pos = self.base_pos + self.w_R_b.dot(np.array([0.0, visual_base_width / 2, 0.0]))
            self.hoist_distance = np.linalg.norm(self.hoist_l_pos - self.hoist_r_pos)
        else:
            # Physical post-homing estimate, aligned with alpine_odometry_node.py.
            # l_1/l_2 are absolute physical lengths because callbacks now apply
            # home offsets and signs.
            self.anchor_pos = self.anchor_left_xyz.copy()
            self.anchor_pos2 = self.anchor_right_xyz.copy()
            self.anchor_distance_y = float(self.anchor_right_xyz[1] - self.anchor_left_xyz[1])

            rope_dir_l = self.axis_vector_from_rot(self.w_R_rope, self.left_rope_axis)
            x_rope_l_attach = self.anchor_pos + rope_dir_l * self.l_1
            self.base_pos = x_rope_l_attach + self.w_R_b.dot(self.body_origin_from_left_attachment)

            self.hoist_l_pos = x_rope_l_attach
            self.hoist_r_pos = self.hoist_l_pos + self.w_R_b.dot(self.right_attachment_from_left_body)
            self.hoist_distance = np.linalg.norm(self.hoist_l_pos - self.hoist_r_pos)

        # debug
        #self.base_pos = np.array([1.5, 2.5, 16])

        self.base_rpy = self.math_utils.rot2eul(self.w_R_b)
        # compute ee position  in the world frame
        self.x_ee = self.base_pos - self.w_R_b[0] * self.leg_length

        norm_l = np.linalg.norm(self.hoist_l_pos - self.anchor_pos)
        norm_r = np.linalg.norm(self.hoist_r_pos - self.anchor_pos2)
        self.rope_direction = (self.hoist_l_pos - self.anchor_pos) / max(norm_l, 1e-9)
        self.rope_direction2 = (self.hoist_r_pos - self.anchor_pos2) / max(norm_r, 1e-9)

        self.mat2Gazebo = self.anchor_pos
        self.base_pos_mat = self.base_pos - self.mat2Gazebo

        # compute missin state variable phi / psi_d
        # the psi variable is the extrinsic pitch wrt the world Y axis obtained expanding w_R_rope with extrinsic formula = Rx Ry Rz
        self.psi = math.atan2(self.w_R_rope[0, 2],
                              np.sqrt(math.pow(self.w_R_rope[0, 0], 2) + math.pow(self.w_R_rope[0, 1], 2)))
        # to get the derivative I need also the
        extr_roll = math.atan2(-self.w_R_rope[1, 2], self.w_R_rope[2, 2])
        self.psi_d = np.cos(extr_roll) * self.w_omega_b[1] - np.sin(extr_roll) * self.w_omega_b[2]

        # now that we have also psid we can  estimate base velocity (in WF)
        self.base_vel = self.estimateRobotVelFromStates(self.l_1, self.l_2, self.psi, self.l_1d, self.l_2d, self.psi_d)

        # use geometric intuition for psid
        n_par = (self.anchor_pos - self.anchor_pos2) / np.linalg.norm(self.anchor_pos - self.anchor_pos2)
        rope2_axis = (self.base_pos - self.anchor_pos2) / np.linalg.norm(self.base_pos - self.anchor_pos2)
        self.n_bar = np.cross(n_par, rope2_axis) / np.linalg.norm(np.cross(n_par, rope2_axis))

        # I should not publihsh base_link tf cause is a fixed based robot I just need to publish joints and comoute TFS with robot state publisher
        # self.broadcaster.sendTransform(self.base_pos, self.body_imu_orientation, ros.Time.now(), '/base_link', '/world')

        # rviz
        self.q_des = self.solver.computeJointVariables(self.base_pos, self.w_R_b, self.q_des_q0, debug=False)

        # Force the two prismatic rope joints to the measured effective lengths.
        # This makes RViz show the actual current rope extension, not only the IK
        # value reconstructed from the body pose.
        if getattr(self, 'rviz_use_measured_rope_joints', True):
            try:
                self.q_des[self.rope_index[0]] = self.q[self.rope_index[0]]
                self.q_des[self.rope_index[1]] = self.q[self.rope_index[1]]
            except Exception:
                pass

        msg = JointState()
        msg.name = self.joint_names
        msg.header.stamp = ros.Time.now()  # ros.Time.from_sec(self.time)
        msg.position = self.q_des
        self.pub_joints.publish(msg)

        self.publish_rope_length_topics_and_text()

    def initVars(self):

        self.n_joints = len(conf.robot_params[self.robot_name]['joint_names'])
        self.rope_length_log = np.empty((2, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q = np.zeros(self.n_joints)
        self.qd = np.zeros(self.n_joints)
        self.tau = np.zeros(self.n_joints)
        self.q_des = np.zeros(self.n_joints)
        self.qd_des = np.zeros(self.n_joints)
        self.tau_ffwd = np.zeros(self.n_joints)
        self.l_1 = 0
        self.l_2 = 0
        self.l_1d = 0
        self.l_2d = 0

        # Until the homing/rope_zero sequence is finished, keep the exact old
        # Locosim/RViz convention.  Physical home offsets are activated only
        # after homing, so at power-on the robot appears in the same homing pose
        # as before this offset patch.
        self.rope_offsets_active = False
        self._alpine_homing_completed = False

        # ALPINE real-robot rope/odometry calibration.
        # Startup RViz uses the historical home visualization width; after
        # homing, physical_hoist_distance is taken from the body geometry params.
        self.base_width = 0.4
        self.hoist_distance = self.base_width
        self.physical_hoist_distance = 0.55
        self.initRopeOffsets()
        if getattr(self, 'pipeline_position_then_jump_enabled', False):
            self.landing = bool(getattr(self, 'pipeline_landing_enabled', False))

        self.g = np.zeros(self.n_joints)
        self.x_ee = np.zeros(3)
        self.x_ee_des = np.zeros(3)

        self.contactForceW = np.zeros(3)
        self.contactMomentW = np.zeros(3)

        self.time = 0.
        self.rope_l_imu_orientation = np.array([0, 0, 0, 1])
        self.rope_l_imu_angular_velocity = np.zeros((3))
        self.rope_l_imu_rpy = np.zeros((3))
        self.rope_l_imu_rpy_d = np.zeros((3))
        self.body_imu_orientation = np.array([0, 0, 0, 1])
        self.body_imu_rpy = np.zeros((3))
        self.body_imu_angular_velocity = np.zeros((3))
        self.w_base_vel = np.zeros((3))  # TODO implement in odometry

        # log vars
        self.q_des_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_des_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.qd_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_ffwd_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.tau_log = np.empty((self.n_joints, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.x_ee_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.x_ee_des_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.log_counter = 0

        self.rope_left_length = 0.0
        self.rope_right_length = 0.0
        self.rope_left_raw_length = 0.0
        self.rope_right_raw_length = 0.0

        self.prejump_left_ref_abs = float('nan')
        self.prejump_right_ref_abs = float('nan')
        self.prejump_position_start_time = 0.0
        self.prejump_position_command_sent = False
        self.prejump_position_print_counter = 0
        self.prejump_lateral_sine_start_time = 0.0
        self.prejump_lateral_sine_print_counter = 0
        self.prejump_position_settle_start_time = 0.0
        self.prejump_position_settle_print_counter = 0
        self.jump_body_command_sent = False
        self.manual_jump_service_called = False
        self.last_jump_energy = float('nan')

        self.qdd_des = np.zeros(self.n_joints)
        self.base_accel = np.zeros(3)
        self.base_rpy = np.zeros(3)
        self.Fr_l_fbk = 0
        self.Fr_r_fbk = 0
        self.Fr_l = 0
        self.Fr_r = 0
        self.prop_force_x = 0
        self.touch_down_detected_l = False
        self.touch_down_detected_r = False
        self.optimal_control_traj_finished = False
        self.MPC_tracking_error = []

        # init new logged vars here
        self.com_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.simp_model_state_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        # self.ldot_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.base_pos_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_rpy_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']
        self.time_jump_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_l_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.Fr_r_fbk_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_1d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.l_2d_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.psid_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.base_vel_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan
        self.prop_force_x_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        self.contactForceW_log = np.empty((3, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        w_R_wall = self.math_utils.eul2Rot(np.array([0, -conf.robot_params[p.robot_name]['wall_inclination'], 0]))
        self.wall_normal = w_R_wall[:,
                           0].copy()  # take X axis, I need to use copy otherwise matlab complains is not contiguous

        self.mpc_index = 0
        self.mpc_index_old = 0
        self.mpc_index_ffwd = 0  # updated only when we stop recomputing mpc

        self.targetReceived = True  # in sim just give hardcoded target

        self.prop_thrusts = [0] * 4
        self.prop_thrusts_log = np.empty((4, conf.robot_params[self.robot_name]['buffer_size'])) * nan

        propeller_orient = np.array([0.25 * np.pi, 0.75 * np.pi, np.pi + 0.25 * np.pi, np.pi + 0.75 * np.pi])
        self.orientControl = OrientationController(base_line_x = 0.1, base_line_y = 0.2, propeller_orient=propeller_orient)

        self.prop_force_y = 0.0
        self.prop_force_y_log = np.empty((conf.robot_params[self.robot_name]['buffer_size'])) * nan

        #rviz
        from closed_loop_inverse_kinematics import ClosedLoopKinSolver
        self.solver = ClosedLoopKinSolver(robot_name=self.robot_name)

    def logData(self):
        if (self.log_counter < conf.robot_params[self.robot_name]['buffer_size']):
            self.simp_model_state_log[:, self.log_counter] = np.array([self.psi, self.l_1, self.l_2])
            # self.ldot_log[self.log_counter] = self.ldot
            self.base_pos_log[:, self.log_counter] = self.base_pos
            self.base_rpy_log[:, self.log_counter] = self.base_rpy
            self.Fr_l_log[self.log_counter] = self.Fr_l
            self.Fr_r_log[self.log_counter] = self.Fr_r
            self.Fr_l_fbk_log[self.log_counter] = self.Fr_l_fbk
            self.Fr_r_fbk_log[self.log_counter] = self.Fr_r_fbk
            self.l_1d_log[self.log_counter] = self.l_1d
            self.l_2d_log[self.log_counter] = self.l_2d
            self.psid_log[self.log_counter] = self.psi_d
            self.base_vel_log[:, self.log_counter] = self.w_base_vel

            self.prop_force_x_log[self.log_counter] = self.prop_force_x
            self.prop_thrusts_log[:, self.log_counter] = self.prop_thrusts
            self.prop_force_y_log[self.log_counter] = self.prop_force_y

            self.rope_length_log[:, self.log_counter] = np.array([
                self.rope_left_length,
                self.rope_right_length
            ])
            # self.time_jump_log[self.log_counter] = self.time - self.end_thrusting

        super().logData()

    def deregister_node(self):
        super().deregister_node()
        # keep the master alive
        # os.system(" rosnode kill -a")
        # os.system(" pkill rosmaster")
        os.system("rosnode kill /robot_state_publisher  ")
        os.system("rosnode kill   /rviz")

    def plotStuff(self):
        print("PLOTTING")
        print(colored(
            "The initial p0_x and mountain_pitch can be different by the desired ones computed by optim, even if we started optim from actual p0, "
            "because the robot sags a bit due to leg reorientation", "red"))
        # plotFrameLinear('com position', 1, p.time_log, None, p.com_log)
        # plotFrameLinear('contact force', 2, p.time_log, None, p.contactForceW_log)
        actual_com = p.base_pos_log - p.mat2Gazebo.reshape(3, 1)  # mat2Gazebo is WF in matlab
        time_gazebo = p.time_log - p.start_logging
        # plotJoint('position', time_gazebo, p.q_log, p.q_des_log, joint_names=conf.robot_params[p.robot_name]['joint_names'])
        # plot3D('basePos', 2,  ['X', 'Y', 'Z'], time_gazebo, actual_com, p.ref_time, p.ref_com)
        # plot3D('matlab states', 3, ['psi', 'l1', 'l2'], time_gazebo, p.simp_model_state_log, p.ref_time, np.vstack((p.ref_psi, p.ref_l_1, p.ref_l_2)) )
        plot3D('matlab states', 3, ['psi', 'l1', 'l2'], p.time_log, p.simp_model_state_log)
        plot3D(
            'winch rope lengths',
            4,
            ['left rope', 'right rope', 'unused'],
            p.time_log,
            np.vstack((
                p.rope_length_log[0, :],
                p.rope_length_log[1, :],
                np.zeros_like(p.time_log)
            ))
        )
        plt.figure()
        plt.title("lateral propeller command fy")
        plt.plot(p.time_log, p.prop_force_y_log)
        plt.grid()
        # plot rope forces
        # plt.figure()
        # plt.subplot(2, 1, 1)
        # plt.ylabel("Fr_l")
        # plt.plot(p.ref_time, p.Fr_l0, color='red')
        # plt.plot(time_gazebo, p.Fr_l_log, color='blue')
        # plt.grid()
        # plt.subplot(2, 1, 2)
        # plt.ylabel("Fr_r")
        # plt.plot(p.ref_time, p.Fr_r0, color='red')
        # plt.plot(time_gazebo, p.Fr_r_log, color='blue')
        # plt.grid()

        # plot propeller thrusts
        # plt.figure()
        # plt.ylabel("prop_thrusts")
        # plt.subplot(4, 1, 1)
        # plt.grid()
        # plt.plot(p.time_log, p.prop_thrusts_log[0,:], label="prop1", color='blue')
        # plt.subplot(4, 1, 2)
        # plt.grid()
        # plt.plot(p.time_log, p.prop_thrusts_log[1,:], label="prop2", color='blue')
        # plt.subplot(4, 1, 3)
        # plt.grid()
        # plt.plot(p.time_log, p.prop_thrusts_log[2,:], label="prop3", color='blue')
        # plt.subplot(4, 1, 4)
        # plt.plot(p.time_log, p.prop_thrusts_log[3,:], label="prop4", color='blue')
        # plt.legend()
        # plt.grid()
        # plotFrameLinear('position', time_log=p.time_log, Pose_log=p.base_rpy_log)

        # save data
        # filename = f'test_gazebo_MPC_{p.MPC_control}.mat'
        # mio.savemat(filename, {'ref_time': p.ref_time, 'ref_com': p.ref_com,
        #                         'time_gazebo': time_gazebo, 'actual_com': actual_com,
        #                         'ref_psi':p.ref_psi,'ref_l_1':p.ref_l_1, 'ref_l_2':p.ref_l_2,
        #                         'psi': p.simp_model_state_log[0,:], 'l_1': p.simp_model_state_log[1,:], 'l_2': p.simp_model_state_log[2,:],
        #                         'psid': p.psid_log, 'l_1d': p.l_1d_log,'l_2d': p.l_2d_log,
        #                         'mu': p.mu , 'Fleg': p.Fleg,'Fr_max': p.Fr_max,
        #                         'Fr_l0': p.Fr_l0, 'Fr_r0': p.Fr_r0,
        #                         'Fr_l': p.Fr_l_log, 'Fr_r': p.Fr_r_log })

    def getIndex(self, t):
        try:
            # get index
            a_bool = self.jumps[self.jumpNumber]["time"] >= t
            idx = min([i for (i, val) in enumerate(a_bool) if val]) - 1
            if idx == -1:
                return 0
            else:
                return idx
        except:
            return -1

    def getImpulseAngle(self):
        angle_hip_roll = math.atan2(self.jumps[self.jumpNumber]["Fleg"][1],
                                    self.jumps[self.jumpNumber]["Fleg"][0])
        angle_hip_pitch = math.atan2(self.jumps[self.jumpNumber]["Fleg"][2], self.jumps[self.jumpNumber]["Fleg"][0])
        print(colored(f"Start orienting leg to (pitch, roll)  : {angle_hip_pitch, angle_hip_roll}", "blue"))
        angle_hip_pitch += -1.57
        return angle_hip_pitch, angle_hip_roll

    # compute the passive and rope joints reference from the matlab position referred to a world frame located in between anchors
    def computeJointVariables(self, p):
        # mountain_wire_pitch_l = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_x'], -p[2])
        # mountain_wire_pitch_r = math.atan2(p[0]-conf.robot_params[self.robot_name]['spawn_2x'], -p[2])
        if conf.robot_params[self.robot_name][
            'wall_inclination'] > 0.:  # TODO missing normal in matlab wall_constraint!
            p[0] = (-p[2]) * math.tan(conf.robot_params[self.robot_name][
                                          'wall_inclination'])  # spawn_x is for the anchor point which is shifted wrt the wall
            print(f"adjusting initial position to be consistent with wall: {p}")

        mountain_wire_pitch_l = math.atan2(p[0], -p[2])
        mountain_wire_pitch_r = math.atan2(p[0], -p[2])

        mountain_wire_roll_l = -math.atan2(-p[2], p[1])
        mountain_wire_roll_r = math.atan2(-p[2], self.anchor_distance_y - p[1])
        # this is an approximation cause I shuould compute the real rope lenght considering the hoist distance so this function is only useful for init but it is inaccurate!
        wire_base_prismatic_l = np.linalg.norm(p) - self.anchor_distance_y * 0.5
        wire_base_prismatic_r = math.sqrt(
            p[0] * p[0] + (self.anchor_distance_y - p[1]) * (self.anchor_distance_y - p[1]) + p[2] * p[
                2]) - self.anchor_distance_y * 0.5

        wire_base_roll_l = -mountain_wire_roll_l
        wire_base_roll_r = -mountain_wire_roll_r
        return [mountain_wire_pitch_r, mountain_wire_roll_r, wire_base_prismatic_r, 0., wire_base_roll_r, 0.,
                mountain_wire_pitch_l, mountain_wire_roll_l, wire_base_prismatic_l, 0., wire_base_roll_l, 0.]

    def detectTouchDown(self):
        force_th = 10.  # TODO implement a strategy based on accelerometer
        # if not self.touch_down_detected and (self.wall_normal.dot(self.contactForceW_l) > force_th):
        #     self.touch_down_detected = True
        # if self.touch_down_detected:
        #     print(colored("TouchDown Detected", "blue"))
        #     # sample com pos
        #     self.x_tilde0 =  self.wall_normal.reshape(1, 3) @ (self.com)# - self.x_p)
        #     return True
        # else:
        #     return False

    def resetRope(self):
        print(colored(f"Start Position Mode", "red"))
        # enable PD for rope and reset the PD reference to the new estension
        # sample the new elongation
        self.q_des[p.rope_index[0]] = np.copy(p.q[p.rope_index[0]])
        self.q_des[p.rope_index[1]] = np.copy(p.q[p.rope_index[1]])
        # print("resetting rope joints qdes : ", self.q_des[p.rope_index])
        self.Fr_r = 0.
        self.Fr_l = 0.
        self.tau_ffwd[p.rope_index] = np.zeros(2)
        self.setRopeControlMode('closed_loop_position')

        # Hold the current physical lengths when returning to position mode.
        # The wrapper subtracts the home offsets and applies rope signs before
        # sending the raw winch reference.
        if hasattr(self, 'homingProcedure'):
            self.publish_rope_position_abs("right", self.rope_right_length)
            self.publish_rope_position_abs("left", self.rope_left_length)

    def printParams(self, p0, pf):
        print(colored(f"p0: {p0}", "red"))
        print(colored(f"pf: {pf}", "red"))
        print(colored(f"Fleg_max: {self.Fleg_max}", "red"))
        print(colored(f"Fr_max: {self.Fr_max}", "red"))
        print(colored(f"mu: {self.mu}", "red"))
        print(colored(f"jump_clearance: {self.optim_params['jump_clearance']}", "red"))
        print(colored(f"mass: {self.optim_params['m']}", "red"))
        print(colored(f"num_params: {self.optim_params['num_params']}", "red"))
        print(colored(f"int_method: {self.optim_params['int_method']}", "red"))
        print(colored(f"N_dyn: {self.optim_params['N_dyn']}", "red"))
        print(colored(f"FRICTION_CONE: {self.optim_params['FRICTION_CONE']}", "red"))
        print(colored(f"int_steps: {self.optim_params['int_steps']}", "red"))
        print(colored(f"contact_normal: {self.optim_params['contact_normal']}", "red"))
        print(colored(f"b: {self.optim_params['b']}", "red"))
        print(colored(f"p_a1: {self.optim_params['p_a1']}", "red"))
        print(colored(f"p_a2: {self.optim_params['p_a2']}", "red"))
        print(colored(f"g: {self.optim_params['g']}", "red"))
        print(colored(f"w1: {self.optim_params['w1']}", "red"))
        print(colored(f"w2: {self.optim_params['w2']}", "red"))
        print(colored(f"w3: {self.optim_params['w3']}", "red"))
        print(colored(f"w4: {self.optim_params['w4']}", "red"))
        print(colored(f"w5: {self.optim_params['w5']}", "red"))
        print(colored(f"w6: {self.optim_params['w6']}", "red"))
        print(colored(f"T_th: {self.optim_params['T_th']}", "red"))

    def initOptim(self, p0, pf):
        ##offline optim vars
        self.Fleg_max = 300.
        self.Fr_max = 90.  # had to increas because of slopes downward jumps it used tp be 90
        self.Fr_min = 0.  # had to increas because of slopes downward jumps it used tp be 0
        # down ward jumps
        # self.Fr_max = 190.  # had to increas because of slopes downward jumps it used tp be 90
        # self.Fr_min = 15.  # had to increas because of slopes downward jumps it used tp be 0
        self.mu = 0.8
        self.optim_params = {}

        if self.OBSTACLE_AVOIDANCE == "mesh":
            self.optim_params['m'] = self.getRobotMass()
            self.optim_params['num_params'] = 4.
            self.optim_params['int_method'] = 'rk4'
            self.optim_params['N_dyn'] = 30.
            self.optim_params['FRICTION_CONE'] = 1.
            self.optim_params['int_steps'] = 5.
            self.optim_params['b'] = self.anchor_distance_y
            self.optim_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
            self.optim_params['p_a2'] = matlab.double([0., self.optim_params['b'], 0.]).reshape(3, 1)
            self.optim_params['g'] = 9.81
            self.optim_params['w1'] = 1.  # smooth
            self.optim_params[
                'w2'] = 1.  # hoist work 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
            self.optim_params['w3'] = 0.
            self.optim_params['w4'] = 0.
            self.optim_params['w5'] = 0.
            self.optim_params['w6'] = 0.
            self.optim_params['T_th'] = 0.05
            self.optim_params['obstacle_avoidance'] = 'mesh'
            self.optim_params['jump_clearance'] = 1.
            # Interpolator (note: z must be increasing — here from -10 to 0)
            # correct initial and final positions
            p0[0] = self.terrainManager.wall_surface_eval(p0[2], p0[1], self.mesh_x, self.mesh_y, self.mesh_z)
            pf[0] = self.terrainManager.wall_surface_eval(pf[2], pf[1], self.mesh_x, self.mesh_y, self.mesh_z)
            # does not work non matching with test_mex TODO
            # p0= np.array([0.99103, 2.5, -6.])
            # pf= np.array([0.40632, 4., -4.])

            # compute consistent normal
            normal = self.terrainManager.wall_normal_eval(p0[2], p0[1], self.mesh_x, self.mesh_y, self.mesh_z)
            self.optim_params['mesh_x'] = self.mesh_x
            self.optim_params['mesh_y'] = self.mesh_y
            self.optim_params['mesh_z'] = self.mesh_z
            self.optim_params['contact_normal'] = matlab.double(normal)
        else:
            self.optim_params['jump_clearance'] = 1.
            self.optim_params['m'] = self.getRobotMass()
            # if terrain is inclined we consider only the Y,Z component of the pf and we need to compute a target point consistent with the wall!
            if conf.robot_params[p.robot_name][
                'wall_inclination'] > 0.:  # TODO missing normal in matlab wall_constraint!
                pf[0] = (-pf[2]) * math.tan(conf.robot_params[p.robot_name]['wall_inclination']) + \
                        conf.robot_params[p.robot_name][
                            'spawn_x']  # spawn_x is for the anchor point which is shifted wrt the wall
                print(f"adjusting landing target to be consistent with wall: {pf}")
            # no longer used
            self.optim_params['obstacle_avoidance'] = False
            self.optim_params['obstacle_location'] = matlab.double(np.zeros(3)).reshape(3, 1)
            self.optim_params['obstacle_size'] = matlab.double(np.zeros(3)).reshape(3, 1)
            self.optim_params['num_params'] = 4.
            self.optim_params['int_method'] = 'rk4'
            self.optim_params['N_dyn'] = 30.
            self.optim_params['FRICTION_CONE'] = 1.
            self.optim_params['int_steps'] = 5.
            self.optim_params['contact_normal'] = matlab.double([1, 0, 0]).reshape(3, 1)
            self.optim_params['b'] = self.anchor_distance_y
            self.optim_params['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
            self.optim_params['p_a2'] = matlab.double([0., self.optim_params['b'], 0.]).reshape(3, 1)
            self.optim_params['g'] = 9.81
            self.optim_params['w1'] = 1.  # smooth
            self.optim_params[
                'w2'] = 0.  # hoist work 100.  # hoist work use this for multiple jumps for energetic comparison (test are for 0 or 100)
            self.optim_params['w3'] = 0.
            self.optim_params['w4'] = 0.
            self.optim_params['w5'] = 0.
            self.optim_params['w6'] = 0.
            self.optim_params['T_th'] = 0.05

        try:
            self.matvars = self.eng.optimize_cpp_mex(matlab.double(p0), matlab.double(pf), self.Fleg_max, self.Fr_max,
                                                     self.Fr_min, self.mu, self.optim_params)
        except:
            print(colored("Regenerate matlab code issues in calling optimize_cpp_mex", "red"))
        # extract variables
        self.ref_com = mat_matrix2python(self.matvars['p'])
        self.ref_psi = mat_vector2python(self.matvars['psi'])
        self.ref_l_1 = mat_vector2python(self.matvars['l1'])
        self.ref_l_2 = mat_vector2python(self.matvars['l2'])
        self.ref_time = mat_vector2python(self.matvars['time'])
        self.Fr_l0 = mat_vector2python(self.matvars['Fr_l'])
        self.Fr_r0 = mat_vector2python(self.matvars['Fr_r'])
        self.Fleg = mat_vector2python(self.matvars['Fleg'])
        # this is computed integrating the dynamics with dt and can be different from the reference, we should use the reference at the end of the horizon
        # self.targetPos = mat_vector2python(self.matvars['achieved_target'])
        self.targetPos = self.ref_com[:, -1]  # output of optumization
        self.targetPosIdeal = self.ref_com[:, -1]
        print(colored(f"offline optimization accomplished, p0:{p0}, target(rough integr):{self.targetPos}", "blue"))
        print(colored(f"target to be compared with text_mex_x.py (fine integr. ) is:{self.matvars['achieved_target']}",
                      "blue"))
        self.jumps = [{"time": self.ref_time, "thrustDuration": self.matvars['T_th'], "p0": p0,
                       "targetPos": self.targetPos, "Fleg": self.Fleg,
                       "Fr_r": self.Fr_r0, "Fr_l": self.Fr_l0, "Tf": self.matvars['Tf']}]

        # MPC vars (need to perform before normal optim to know Tf)
        self.mpc_N = int(0.4 * self.optim_params['N_dyn'])
        self.Fr_max_mpc = 100.

        self.optim_params_mpc = {}
        self.optim_params_mpc['int_method'] = 'rk4'
        self.optim_params_mpc['int_steps'] = 5.
        self.optim_params_mpc['contact_normal'] = matlab.double([1., 0., 0.]).reshape(3, 1)
        self.optim_params_mpc['b'] = self.anchor_distance_y
        self.optim_params_mpc['p_a1'] = matlab.double([0., 0., 0.]).reshape(3, 1)
        self.optim_params_mpc['p_a2'] = matlab.double([0., self.optim_params_mpc['b'], 0.]).reshape(3, 1)
        self.optim_params_mpc['g'] = 9.81
        self.optim_params_mpc['m'] = self.getRobotMass()
        self.optim_params_mpc['w1'] = 1.
        self.optim_params_mpc['w2'] = 0.000001
        self.optim_params_mpc['mpc_dt'] = matlab.double(self.matvars['Tf'] / (self.optim_params['N_dyn'] - 1))
        self.deltaFr_l = np.zeros((int(self.mpc_N)))
        self.deltaFr_r = np.zeros((int(self.mpc_N)))
        self.propeller_force = np.zeros((int(self.mpc_N)))

        if self.PLOT_MPC:
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)

    def computeMPC(self, delta_t):
        # after the thrust we start MPC,  it will start from time 0.05 so the index will start from  2
        if self.getIndex(delta_t) != -1:
            self.mpc_index = self.getIndex(delta_t)
        else:  # whenever the MPC should not be updated anymore use delta_t to imncrement mpc_index_ffwd
            # print("delta_t MOD dtMpc", (delta_t % self.optim_params_mpc['mpc_dt']))
            if (delta_t % self.optim_params_mpc['mpc_dt']) < 0.001:  # increment mpc_index_ffwd every mpc_dt
                self.mpc_index_ffwd += 1
                if self.mpc_index_ffwd > (self.mpc_N - 1):  # reference is finished keep the last computed one
                    self.mpc_index_ffwd = self.mpc_N - 1
                # debug
                # print("stop mpc, applying ffwd, mpc_index_ffwd: ", self.mpc_index_ffwd)
        # This is better for const dist cause it keeps optimizing till the end!!!
        if (self.mpc_index != self.mpc_index_old):  # do optim only every dtMPC  not every dt
            # reduce MPC horizon gradually at the end
            if ((self.mpc_index + self.mpc_N) >= len(self.ref_time)):
                self.mpc_N -= 1
            # eval ref
            ref_com = matlab.double(self.ref_com[:, self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_l0 = matlab.double(self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            Fr_r0 = matlab.double(self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N].tolist())
            actual_t = matlab.double(self.ref_time[self.mpc_index])
            actual_state = matlab.double([self.psi, self.l_1, self.l_2, self.psi_d, self.l_1d, self.l_2d]).reshape(6, 1)

            # perform optimization
            x = mat_vector2python(
                self.eng.optimize_cpp_mpc_propellers_mex(actual_state, actual_t, ref_com, Fr_l0, Fr_r0, self.Fr_max_mpc,
                                                         self.mpc_N, self.optim_params_mpc))
            # extract optim vars
            self.deltaFr_l = x[:self.mpc_N]
            self.deltaFr_r = x[self.mpc_N:2 * self.mpc_N]
            self.propeller_force = x[2 * self.mpc_N:3 * self.mpc_N]

            # store tracking error for RMSE computation
            tracking_error = self.ref_com[:, self.mpc_index] - (self.base_pos - p.anchor_pos)
            self.MPC_tracking_error.append(np.linalg.norm(tracking_error))
            # online plot MPC
            if self.PLOT_MPC:
                self.onlinePlotMPC(self.deltaFr_l, self.deltaFr_r)

        self.mpc_index_old = self.mpc_index

        return self.deltaFr_l[self.mpc_index_ffwd], self.deltaFr_r[self.mpc_index_ffwd], self.propeller_force[
            self.mpc_index_ffwd]

    def onlinePlotMPC(self, deltaFr_l, deltaFr_r):
        # debug
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.set_label("delta Frl")
        self.ax2.set_label("delta Frr")
        self.ax1.grid()
        self.ax2.grid()
        # MPC action (red)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_l, "or-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N], deltaFr_r, "or-")
        # full action (black)
        self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_l0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_l, "ok-")
        self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
                      self.Fr_r0[self.mpc_index:self.mpc_index + self.mpc_N] + deltaFr_r, "ok-")
        # plot Fr limits Does not work
        # self.ax1.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
        #               -self.Fr_max * np.ones((1, self.mpc_N)), "r-")
        # self.ax2.plot(self.ref_time[self.mpc_index:self.mpc_index + self.mpc_N],
        #               -self.Fr_max * np.ones((1, self.mpc_N)), "r-")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def computeJumpEnergyConsumption(self):
        # get index
        # a_bool = self.jumps[self.jumpNumber]["time"] > self.jumps[self.jumpNumber]["thrustDuration"]
        # lift_off_idx = min([i for (i, val) in enumerate(a_bool) if val]) - 1
        lift_off_idx = np.max(
            np.where((self.time_log - self.start_logging) <= self.jumps[self.jumpNumber]["thrustDuration"]))
        impulse_work = 0.5 * self.optim_params['m'] * self.base_vel_log[:, lift_off_idx].dot(
            self.base_vel_log[:, lift_off_idx])  # ekin at liftoff
        # this integral is done on a rough discretization dt
        touch_down_idx = np.max(np.where((self.time_log - self.start_logging) < p.jumps[p.jumpNumber]["Tf"]))
        hoist_work = 0.
        for i in range(touch_down_idx):
            hoist_work = hoist_work + (
                    abs(self.Fr_r_log[i] * self.l_2d_log[i]) + abs(self.Fr_l_log[i] * self.l_1d_log[i])) * \
                         conf.robot_params[p.robot_name]['dt']
        return impulse_work + hoist_work

    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
        if self.real_robot:
            msg = RopeCommand()
            msg.rope_force = p.tau_ffwd[p.rope_index[0]]
            msg.rope_force = p.q_des[p.rope_index[0]]
            msg.rope_velocity = p.qd_des[p.rope_index[0]]
            msg.stamp = ros.Time.now()
            self.pub_rope_command_r()
            msg.rope_force = p.tau_ffwd[p.rope_index[1]]
            msg.rope_force = p.q_des[p.rope_index[1]]
            msg.rope_velocity = p.qd_des[p.rope_index[1]]
            self.pub_rope_command_l()


    # REAL ROBOT FUNCTIONS
    def startRealRobot(self):
        # os.system("killall rviz gzserver gzclient")
        print(colored('------------------------------------------------ROBOT IS REAL!', 'blue'))
        checkRosMaster()

        self.leg_length = 0.45
        self.initRopeOffsets()

        # IMPORTANT: keep the historical RViz/Locosim startup pose.
        # These are the same anchor/arganello values used by the old script:
        #   left  = [0.2, 0.0, 2.4]
        #   right = [0.2, 2.2, 2.4]
        # They are only for robot_description/RViz at script startup.
        # The physical odometry anchors remain anchor_left_xyz/anchor_right_xyz
        # and are used after homing for offset calculations.
        self.base_width = float(self.rviz_home_base_width)
        self.hoist_distance = self.base_width
        rviz_left = np.array(self.rviz_home_anchor_left_xyz, dtype=float)
        rviz_right = np.array(self.rviz_home_anchor_right_xyz, dtype=float)

        conf.robot_params[self.robot_name]['spawn_x'] = float(rviz_left[0])
        conf.robot_params[self.robot_name]['spawn_y'] = float(rviz_left[1])
        conf.robot_params[self.robot_name]['spawn_z'] = float(rviz_left[2])
        conf.robot_params[self.robot_name]['spawn_2x'] = float(rviz_right[0])
        conf.robot_params[self.robot_name]['spawn_2y'] = float(rviz_right[1])
        conf.robot_params[self.robot_name]['spawn_2z'] = float(rviz_right[2])

        print(colored(
            f"RViz startup home anchors: left={rviz_left.tolist()}, right={rviz_right.tolist()}, "
            f"visual_base_width={self.base_width:.3f}",
            "blue"
        ))

        # loads robot_description
        additional_urdf_args = ' spawn_x:=' + str(conf.robot_params[self.robot_name]['spawn_x'])
        additional_urdf_args += ' spawn_y:=' + str(conf.robot_params[self.robot_name]['spawn_y'])
        additional_urdf_args += ' spawn_z:=' + str(conf.robot_params[self.robot_name]['spawn_z'])
        additional_urdf_args += ' spawn_2x:=' + str(conf.robot_params[self.robot_name]['spawn_2x'])
        additional_urdf_args += ' spawn_2y:=' + str(conf.robot_params[self.robot_name]['spawn_2y'])
        additional_urdf_args += ' spawn_2z:=' + str(conf.robot_params[self.robot_name]['spawn_2z'])
        launchFileNode(package='climbingrobot_description', launch_file='upload.launch',
                       additional_args=additional_urdf_args)

        # launches robot state publisher
        startNode(package='robot_state_publisher', executable='robot_state_publisher')
        # start rviz
        startNode(package='rviz', executable='rviz',
                  args='-d ' + rospkg.RosPack().get_path('climbingrobot_description') + '/rviz/conf.rviz')

        # launch hw interface (we launch ouside)
        # launchFileNode(package='climbingrobot_hardware_interface', launch_file='alpine_low_level_bringup.launch')

        print(colored("DONE", "red"))

    def startRealRobotPublisherSubscribers(self):

        self.ros_pub = RosPub(self.robot_name, only_visual=True, markers_time_to_live=0)
        self.broadcaster = tf.TransformBroadcaster()

        # this is for the matlab optim TODO uncomment
        # self.eng = matlab.engine.start_matlab()
        #
        # if self.OBSTACLE_AVOIDANCE == 'mesh':
        #     self.eng.addpath('./codegen_mesh', nargout=0)
        # else:
        #     self.eng.addpath('./codegen', nargout=0)
        #
        # if self.MPC_control:
        #     self.eng.addpath('./codegen_mpc', nargout=0)

        if self.SAVE_BAG:
            self.recorder = RosbagControlledRecorder(record_from_startup_=False)

        self.sub_rope_telemetry_l = ros.Subscriber("/winch/left/telemetry", RopeTelemetry,
                                                   callback=self._receive_rope_telemetry_l, queue_size=1,
                                                   tcp_nodelay=True)
        self.sub_rope_telemetry_r = ros.Subscriber("/winch/right/telemetry", RopeTelemetry,
                                                   callback=self._receive_rope_telemetry_r, queue_size=1,
                                                   tcp_nodelay=True)
        self.pub_rope_command_l = ros.Publisher("/winch/left/command", RopeCommand, queue_size=1, tcp_nodelay=True)
        self.pub_rope_command_r = ros.Publisher("/winch/right/command", RopeCommand, queue_size=1, tcp_nodelay=True)
        self.rope_control_mode_l = ros.ServiceProxy('/winch/left/set_control_mode', RopeControlMode)
        self.rope_control_mode_r = ros.ServiceProxy('/winch/right/set_control_mode', RopeControlMode)
        # to orchestrator
        self.sub_des_target = ros.Subscriber("/planner/desired_target", geometry_msgs.msg.Vector3,
                                             callback=self._receive_target, queue_size=1, tcp_nodelay=True)
        self.pub_goal_status = ros.Publisher("/planner/goal_status", std_msgs.msg.String, queue_size=1,
                                             tcp_nodelay=True)

        # communication to alpine
        self.sub_alpine_telemetry = ros.Subscriber("/alpine_body/telemetry", AlpineBodyTelemetry,
                                                   callback=self._receive_alpine_telemetry, queue_size=1,
                                                   tcp_nodelay=True)
        self.pub_alpine_wrench = ros.Publisher("/alpine_body/wrench_cmd", Wrench, queue_size=1, tcp_nodelay=True)
        self.pub_alpine_cmdraw = ros.Publisher("/alpine_body/cmd_raw", std_msgs.msg.String, queue_size=1,
                                               tcp_nodelay=True)
        self.alpine_command_service = ros.ServiceProxy('/alpine_body/command', AlpineBodyCommand)  # optimized path
        self.manual_jump_service = ros.ServiceProxy('/alpine/jump', Trigger)  # manual JumpNode path

        # communication to alpine
        self.pub_propeller_command = ros.Publisher("/alpine_body/propeller_command", PropellerCommand, queue_size=1, tcp_nodelay=True)

        # for rviz

        self.pub_joints = ros.Publisher("/joint_states", JointState, queue_size=1, tcp_nodelay=True)
        self.pub_rope_length_marker = ros.Publisher("/alpine/actual_rope_lengths_marker", Marker, queue_size=10)
        self.pub_rope_left_abs_length = ros.Publisher("/alpine/left_rope_abs_length_m", std_msgs.msg.Float32, queue_size=10)
        self.pub_rope_right_abs_length = ros.Publisher("/alpine/right_rope_abs_length_m", std_msgs.msg.Float32, queue_size=10)

        # Make RViz see the same home configuration immediately at script start,
        # before the blocking winch homing sequence begins.
        self.publish_initial_homing_pose_for_rviz(duration_s=self.rviz_home_publish_s)

        # Homing.
        # During this whole sequence rope_offsets_active stays False, so RViz uses
        # the original home-relative Locosim convention and the robot appears at
        # startup/homing exactly as before.
        self.rope_offsets_active = False
        self._alpine_homing_completed = False
        self.initRopeOffsets()
        self.homingProcedure = WinchStartupSequence()
        self.homingProcedure.run_sequence()

        # From here onward rope_zero is done.  Activate the physical offset
        # convention and use it for all position references.
        self.rope_offsets_active = True
        self._alpine_homing_completed = True
        self.base_width = float(getattr(self, 'physical_hoist_distance', self.base_width))
        self.hoist_distance = self.base_width

        # Recompute the latest stored telemetry values in the physical convention
        # immediately, instead of waiting for the next telemetry packet.
        self.rope_left_length = self.rope_abs_length("left", self.rope_left_raw_length)
        self.rope_right_length = self.rope_abs_length("right", self.rope_right_raw_length)
        self.l_1 = self.rope_left_length
        self.l_2 = self.rope_right_length

        self.homingProcedure.publish_mode("closed_loop_position")
        ros.sleep(0.05)

        # Enter position mode safely.  In the new pipeline we first hold the
        # current measured absolute lengths; the 1.20 m pre-jump descent is
        # commanded later by stateMachineLoop(), where it can be monitored.
        # Legacy behavior can be restored by setting
        # /alpine/pipeline_position_then_jump_enabled:=false.
        if getattr(self, 'pipeline_position_then_jump_enabled', True):
            self.publish_current_rope_position_hold()
            hold_s = max(0.0, float(getattr(self, 'post_homing_position_hold_s', 0.50)))
            if hold_s > 0.0:
                print(colored(
                    f"POST-HOMING POSITION HOLD: holding current rope lengths for {hold_s:.2f} s",
                    "yellow"
                ))
                ros.sleep(hold_s)
        else:
            # Move to a physical absolute length after homing.
            # Internally publish_rope_position_abs() sends raw winch references:
            #   raw_ref = sign * (absolute_ref - home_offset)
            self.publish_rope_position_abs("right", self.right_home_offset_m + self.homing_test_delta_m)
            self.publish_rope_position_abs("left", self.left_home_offset_m + self.homing_test_delta_m)

    def _receive_rope_telemetry_l(self, msg):
        self.Fr_l_meas = msg.rope_force
        self.rope_left_raw_length = float(msg.rope_length)

        if not getattr(self, 'rope_offsets_active', False):
            # Old startup/homing convention: no physical home offset yet.
            self.rope_left_length = float(msg.rope_length)
            self.l_1 = self.rope_left_length
            self.l_1d = float(msg.rope_velocity)
        else:
            # Post-homing convention: physical absolute length.
            self.rope_left_length = self.rope_abs_length("left", msg.rope_length)
            self.l_1 = self.rope_left_length
            self.l_1d = self.rope_abs_velocity("left", msg.rope_velocity)

        self.brake_status_l = msg.brake_status
        self.q[self.rope_index[1]] = self.l_1 + self.hoist_distance / 2 - self.anchor_distance_y / 2
        self.qd[self.rope_index[1]] = self.l_1d

    def _receive_rope_telemetry_r(self, msg):
        self.Fr_r_meas = msg.rope_force
        self.rope_right_raw_length = float(msg.rope_length)

        if not getattr(self, 'rope_offsets_active', False):
            # Old startup/homing convention: the right winch was already inverted.
            self.rope_right_length = -float(msg.rope_length)
            self.l_2 = self.rope_right_length
            self.l_2d = -float(msg.rope_velocity)
        else:
            # Post-homing convention: physical absolute length.
            self.rope_right_length = self.rope_abs_length("right", msg.rope_length)
            self.l_2 = self.rope_right_length
            self.l_2d = self.rope_abs_velocity("right", msg.rope_velocity)

        self.brake_status_r = msg.brake_status
        self.q[self.rope_index[0]] = self.l_2 + self.hoist_distance / 2 - self.anchor_distance_y / 2
        self.qd[self.rope_index[0]] = self.l_2d

    def _receive_target(self, msg):
        self.target = np.array([msg.x, msg.y, msg.z])
        self.targetReceived = True
        print(colored(f"received target {self.target}", "red"))

    def _receive_alpine_telemetry(self, msg):
        # rope
        self.rope_l_imu_orientation = np.array([
            msg.rope_imu_orientation.x,
            msg.rope_imu_orientation.y,
            msg.rope_imu_orientation.z,
            msg.rope_imu_orientation.w
        ])
        self.rope_l_imu_angular_velocity = np.array(
            [msg.rope_imu_angular_velocity.x, msg.rope_imu_angular_velocity.y, msg.rope_imu_angular_velocity.z])
        self.rope_l_imu_rpy = np.array([msg.rope_imu_rpy.x, msg.rope_imu_rpy.y, msg.rope_imu_rpy.z])
        self.rope_l_imu_rpy_d = np.array([msg.rope_imu_rpy_d.x, msg.rope_imu_rpy_d.y, msg.rope_imu_rpy_d.z])
        # body
        self.body_imu_orientation = np.array([
            msg.body_imu_orientation.x,
            msg.body_imu_orientation.y,
            msg.body_imu_orientation.z,
            msg.body_imu_orientation.w
        ])
        self.body_imu_rpy = np.array([msg.body_imu_rpy.x, msg.body_imu_rpy.y, msg.body_imu_rpy.z])
        self.body_imu_angular_velocity = np.array(
            [msg.body_imu_angular_velocity.x, msg.body_imu_angular_velocity.y, msg.body_imu_angular_velocity.z])

    def print_message(self, message="", decimate=1000):
        if not hasattr(self, 'print_counter'):
            self.print_counter = 0
        if np.mod(self.print_counter, decimate) == 0:
            print(colored(message, "red"))
        self.print_counter += 1

    def setRopeControlMode(self, mode='idle'):
        req = RopeControlModeRequest()
        req.message = mode
        # Call the service
        try:
            resp = self.rope_control_mode_l(req)
            ros.loginfo("Service response: ack = %s", resp.success)
        except ros.ServiceException as e:
            ros.logerr("Service call failed: %s" % e)
            return False
        try:
            resp = self.rope_control_mode_r(req)
            ros.loginfo("Service response: ack = %s", resp.success)
        except ros.ServiceException as e:
            ros.logerr("Service call failed: %s" % e)
            return False

    def stateMachineLoop(self):
        terminateFlag = False

        # New real-robot pipeline:
        #   1) position mode, command both ropes +prejump_drop_m absolute length
        #   2) wait until the measured effective lengths are reached
        #   3) run a finite lateral sine on /alpine_body/wrench_cmd
        #   4) start the selected jump path
        if p.stateMachine == 'positioning_before_jump':
            status = p.monitor_prejump_position_drop()
            if status == 'done':
                p.start_prejump_position_settle()
            elif status == 'timeout':
                if getattr(p, 'prejump_abort_on_timeout', True):
                    p.publish_current_rope_position_hold()
                    msg = std_msgs.msg.String()
                    msg.data = 'position_timeout'
                    p.pub_goal_status.publish(msg)
                    p.stateMachine = 'position_error'
                    return True
                else:
                    print(colored("PRE-JUMP POSITION timeout -> continuing anyway", "red"))
                    p.start_prejump_position_settle()
            return False

        if p.stateMachine == 'prejump_position_settle':
            status = p.monitor_prejump_position_settle()
            if status == 'done':
                return p.continue_after_prejump_position_settle()
            return False

        if p.stateMachine == 'prejump_lateral_sine':
            status = p.monitor_prejump_lateral_sine()
            if status == 'done':
                return p.start_jump_after_prejump_sequence()
            return False

        if p.stateMachine == 'manual_jump_started':
            # The low-level jump_node owns valves and local rope commands now.
            # Keep this controller alive only for RViz/telemetry updates.
            return False

        if p.stateMachine == 'position_error':
            p.send_alpine_wrench(0.0, 0.0, 0.0)
            return True

        # jump state machine
        if (p.stateMachine == 'idle') and (p.time >= p.startJump):
            # first run optim and fill in jump variable

            p.initOptim(p.base_pos - p.mat2Gazebo, p.target)

            # set the end of thrusting phase
            p.end_thrusting = p.startJump + p.jumps[p.jumpNumber]["thrustDuration"]
            p.start_logging = p.end_thrusting
            p.stateMachine = 'thrusting'  # this phase only waits is not doing anything

            if p.SAVE_BAG:
                p.recorder.start_recording_srv()

            print("\033[34m" + "---------Starting jump  number ", p.jumpNumber, " to optimized target: ",
                  p.jumps[p.jumpNumber]["targetPos"], " from actual p0 : ", p.base_pos - p.mat2Gazebo)
            print(colored(f"Start trusting", "blue"))
            p.tau_ffwd = np.zeros(p.n_joints)
            p.tau_ffwd[p.rope_index] = p.g[p.rope_index]  # compensate gravitu in the virtual joint to go exactly there
            print(colored(f"Start Torque mode", "red"))
            p.setRopeControlMode('close_loop_torque')

            p.stateMachine = 'thrusting'
            p.w_Fleg = p.jumps[p.jumpNumber]["Fleg"]
            p.jump_body_command_sent = False

        if (p.stateMachine == 'thrusting'):

            # service call to send thrust force and contact normal to ALPINE this will initiate the jump
            # Fill the request
            req = AlpineBodyCommandRequest()
            req.leg_force = np.linalg.norm(p.w_Fleg)
            req.contact_normal = geometry_msgs.msg.Vector3(x=p.wall_normal[0], y=p.wall_normal[1], z=p.wall_normal[2])
            # plot Fleg
            p.ros_pub.add_arrow(p.x_ee, np.linalg.norm(p.w_Fleg) * p.wall_normal / p.force_scale, "red", scale=2.5)

            # Apply leg impulse once.  The low-level jump node owns the valve
            # timeline after this service call; calling it every controller tick
            # would restart the valve sequence repeatedly.
            if not getattr(p, 'jump_body_command_sent', False):
                try:
                    resp = p.alpine_command_service(req)
                    ros.loginfo("Service response: ack = %s", resp.ack)
                    p.jump_body_command_sent = bool(resp.ack)
                except ros.ServiceException as e:
                    ros.logerr("Service call failed: %s" % e)

            # During piston thrust keep propeller bias at zero.
            p.send_alpine_wrench(0.0, 0.0, 0.0)

            # Keep ropes neutral during the piston thrust.  The optimized rope
            # force timeline is indexed from lift-off (delta_t=0), so it starts
            # in the flying states below, after p.end_thrusting.
            delta_t = p.time - p.end_thrusting
            p.Fr_r = 0.0
            p.Fr_l = 0.0

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            p.publish_rope_torque_forces(left_force=p.Fr_l, right_force=p.Fr_r)

            if (p.time > p.end_thrusting):
                print(colored("Stop Trhusting", "blue"))
                p.tau_ffwd[p.leg_index] = np.zeros(len(p.leg_index))
                # retract leg for landing
                if p.landing:
                    p.stateMachine = 'flying_and_wait_for_touchdown'
                else:
                    p.stateMachine = 'flying'
                print(colored("Start " + p.stateMachine, "blue"))

        if (p.stateMachine == 'flying'):
            # after the thrust we start MPC it will start from time 0.05 so the index should be 12
            # applying forces to ropes
            delta_t = p.time - p.end_thrusting
            if p.MPC_control:
                # compute orientation control TODO
                # p.prop_force_x, p.prop_force_y, p.prop_moment_z = computeOrientationControl()
                deltaFr_l0, deltaFr_r0, p.prop_force_x = p.computeMPC(delta_t)
                #p.apply_propeller_command(p.prop_force_x, p.prop_force_y, p.prop_moment_z)
                #p.send_alpine_wrench(fx=p.prop_force_x, fy=0.0, mz=0.0)

            else:
                deltaFr_l0 = 0.
                deltaFr_r0 = 0.
                p.send_alpine_wrench(0.0, 0.0, 0.0)

            p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)] + deltaFr_l0
            p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)

            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            p.publish_rope_torque_forces(left_force=p.Fr_l, right_force=p.Fr_r)
            end_flying = p.startJump + p.jumps[p.jumpNumber]["Tf"]

            if (p.time >= end_flying):
                print(colored("Stop Flying", "blue"))
                # reset the qdes
                # we need to reset the rope PD because the Fr are finished and I would get the final value repeated  that is not the good thing to do
                # this will start again the position loop
                p.resetRope()
                energy = p.computeJumpEnergyConsumption()
                p.last_jump_energy = energy
                p.jumpNumber += 1
                if (p.jumpNumber < p.numberOfJumps):
                    p.stateMachine = 'idle'
                    # reset for multiple jumps
                    p.startJump = p.time
                else:

                    p.printLandingInfo()
                    if p.SAVE_BAG:
                        p.recorder.stop_recording_srv()
                    terminateFlag = True

            return terminateFlag
        # this is the same as flying but with the lander
        if (p.stateMachine == 'flying_and_wait_for_touchdown'):
            # applying forces to ropes, when time is finished just rset rope length (only once!) and wait for tf
            delta_t = p.time - p.end_thrusting

            if p.MPC_control:
                deltaFr_l0, deltaFr_r0, p.prop_force_x = p.computeMPC(delta_t)
                prop_forceW = p.n_bar * p.prop_force_x
                # Real robot path: onboard ESP32 closes pitch/yaw loops from the body IMU.
                # The ROS controller only sends a body wrench bias.
                # compute orientation control TODO
                # compute thrust for orientation
                # p.prop_thrusts, w_wrench = p.orientControl.computeThrust(des_orient=np.array([0, 0, 0.7]),
                #                                                          act_orient=p.base_rpy,
                #                                                          w_omega_b=p.w_omega_b,
                #                                                          Ko=conf.robot_params[p.robot_name]['Ko'],
                #                                                          Do=conf.robot_params[p.robot_name]['Do'], w_additional_force=prop_forceW)
                #p.apply_propeller_command(p.prop_thrusts)
                #p.send_alpine_wrench(fx=p.prop_force_x, fy=0.0, mz=0.0)
            else:
                deltaFr_l0 = 0.
                deltaFr_r0 = 0.
                p.send_alpine_wrench(0.0, 0.0, 0.0)

            if not p.optimal_control_traj_finished:
                if p.getIndex(delta_t) == -1:
                    p.optimal_control_traj_finished = True
                    print(colored("Trajectory finished, expecting delayed TD", "blue"))
                    # start again pid gains and reset qdes
                    p.resetRope()

                else:
                    p.Fr_l = p.jumps[p.jumpNumber]["Fr_l"][p.getIndex(delta_t)] + deltaFr_l0
                    p.Fr_r = p.jumps[p.jumpNumber]["Fr_r"][p.getIndex(delta_t)] + deltaFr_r0

                # check for early td and in case reset rope
                if p.detectTouchDown():
                    p.resetRope()
                    print(colored("Early TD detected, Start landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time
            else:  # you are checking for delayed TD you have already reset rope and restored PD
                if p.detectTouchDown():
                    print(colored("Start landing", "blue"))
                    p.stateMachine = 'landing'
                    p.start_landing = p.time

            # plot rope forces
            p.ros_pub.add_arrow(p.hoist_l_pos, p.rope_direction * (p.Fr_l) / p.force_scale, "red", scale=2.5)
            p.ros_pub.add_arrow(p.hoist_r_pos, p.rope_direction2 * (p.Fr_r) / p.force_scale, "red", scale=2.5)
            p.tau_ffwd[p.rope_index[0]] = p.Fr_r
            p.tau_ffwd[p.rope_index[1]] = p.Fr_l
            p.publish_rope_torque_forces(left_force=p.Fr_l, right_force=p.Fr_r)

        if (p.stateMachine == 'landing'):
            print(colored("Start landing", "blue"))
            p.prop_force = (-25.)  # push against the wall
            #TODO
            # p.apply_propeller_force(p.prop_force)
            # p.send_alpine_wrench(0.0, 0.0, 0.0)
            landing_error = p.printLandingInfo()
            msg = std_msgs.msg.String()
            if np.linalg.norm(landing_error) < 0.5:
                msg.data = 'achieved'
            else:
                msg.data = 'error'
            p.pub_goal_status.publish(msg)

            ####TODO
            pass

    def printLandingInfo(self):
        landing_location = self.base_pos - self.mat2Gazebo
        target = getattr(self, 'targetPos', getattr(self, 'target', np.zeros(3)))
        print(colored(f" real landing (in matlab convention) is: {landing_location}", "blue"))
        print(colored(f" while from optim it should be  {target}", "blue"))

        landing_error = target - landing_location
        print(colored(f" the landing error is  {np.linalg.norm(landing_error)}", "blue"))

        try:
            p0_ref = self.jumps[self.jumpNumber]["p0"]
            jump_length = max(np.linalg.norm(p0_ref[:2] - target[:2]), 1e-9)
        except Exception:
            jump_length = 1.0

        if len(getattr(self, 'MPC_tracking_error', [])) > 0:
            MSE = np.square(np.array(self.MPC_tracking_error)).mean()
            RMSE = math.sqrt(MSE)
        else:
            RMSE = float('nan')

        print(colored(
            f" the relative landing error (norm per jump length) is {100 * np.linalg.norm(landing_error) / jump_length}%",
            "blue"
        ))
        print(colored(f" the energy consumption is  {getattr(self, 'last_jump_energy', float('nan'))}", "blue"))
        print(colored(f" the rmse of MPC tracking error is  {RMSE}", "blue"))
        if hasattr(self, 'Fleg'):
            print(colored(f" the leg impulse is  {self.Fleg}", "blue"))
            print(colored(f" the norm of the leg impulse is  {np.linalg.norm(self.Fleg)}", "blue"))
        self.plotStuff()
        return landing_error


def talker(p):
    p.start()

    # Init UNA SOLA VOLTA, prima di subscriber/homing
    p.initVars()
    p.q_des = np.copy(p.q_des_q0)

    p.startRealRobot()
    p.startRealRobotPublisherSubscribers()

    # jump params
    # jump starting position
    p0 = np.array([0.28, 2.5, -6.10104])  # there is singularity for px = 0!
    # jump landing position
    p.target = np.array([0.28, 4, -4])

    # loop frequency
    rate = ros.Rate(1 / conf.robot_params[p.robot_name]['dt'])

    ros.sleep(0.5)
    p.updateKinematicsDynamics()

    p.enable_attitude_hold()

    p.startJump = 2.5
    if getattr(p, 'pipeline_position_then_jump_enabled', True):
        p.stateMachine = 'positioning_before_jump'
    else:
        p.stateMachine = 'idle'
    p.jumpNumber = 0
    p.numberOfJumps = 1
    p.start_logging = 0

    # set initial desired joints for visualization/model
    p.q_des[:12] = p.computeJointVariables(p0)

    # NON richiamare position mode qui: lo fa già la homing procedure
    # p.setRopeControlMode('close_loop_position')

    while True:
        if p.targetReceived:
            print(colored(f"---------------Ideal Target landing: {p.target}", "green"))
            break
        else:
            p.print_message("waiting for target", decimate=1000)
            rate.sleep()

    while not ros.is_shutdown():

        # update the kinematics
        p.updateKinematicsDynamics()

        # Default safe wrench.  The state machine overrides this only during
        # the finite pre-jump lateral sine phase.
        p.send_alpine_wrench(fx=0.0, fy=0.0, mz=0.0)
        p.prop_force_y = 0.0

        stop = p.stateMachineLoop()
        if stop:
            break

        # plot ropes as green arrows only when you not save bags because they are ugly
        if not p.SAVE_BAG:
            p.ros_pub.add_arrow(p.anchor_pos, (p.hoist_l_pos - p.anchor_pos), "green", scale=3.)
            p.ros_pub.add_arrow(p.anchor_pos2, (p.hoist_r_pos - p.anchor_pos2), "green", scale=3.)

        # plot contact force on retractable leg
        p.ros_pub.add_arrow(p.x_ee, p.contactForceW / p.force_scale, "blue", scale=2.5)

        #plot target position (whenever is available)
        try:
            p.ros_pub.add_marker(p.mat2Gazebo + p.jumps[p.jumpNumber]["targetPos"], color="red", radius=0.3, alpha=1.)
            p.ros_pub.add_marker(p.mat2Gazebo + p.targetPosIdeal, color="green", radius=0.5, alpha=0.5)
        except:
            pass

        p.ros_pub.add_marker(p.x_ee, radius=0.05)
        #TODO
        #p.ros_pub.add_mesh(mesh_path=os.environ['LOCOSIM_DIR'] + '/robot_descriptions/climbingrobot_description/meshes/runtime_mesh.obj', position=p.mat2Gazebo, color=None, alpha=1.0)
        p.ros_pub.publishVisual(delete_markers=False)

        p.time = round(p.scalar_time() + conf.robot_params[p.robot_name]['dt'], 4)  # scalar; avoids 1-element-array comparisons
        if (p.time > p.start_logging):
            p.logData()
        # wait for synconization of the control loop
        rate.sleep()

def plot3D(name, figure_id, label, time_log, var, time_mat = None, var_mat = None):
    fig = plt.figure()
    fig.suptitle(name, fontsize=20)
    plt.subplot(3, 1, 1)
    plt.ylabel(label[0])
    plt.plot(time_log, var[0, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[0, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    plt.grid(True)
    plt.legend(['act', 'ref'])

    plt.subplot(3, 1, 2)
    plt.ylabel(label[1])
    plt.plot(time_log, var[1, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[1, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])

    plt.subplot(3, 1, 3)
    plt.ylabel(label[2])
    plt.plot(time_log, var[2, :], linestyle='-', marker="o", markersize=0, lw=5, color='blue')
    if (var_mat is not None):
        plt.plot(time_mat, var_mat[2, :], linestyle='-', marker="o", markersize=0, lw=5, color='red')
    plt.grid()
    plt.legend(['act', 'ref'])


if __name__ == '__main__':
    p = ClimbingrobotController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()

    finally:
        ros.signal_shutdown("killed")
        p.deregister_node()
        if p.landing:  # for the landing test you should press Ctrl C to stop everything
            p.plotStuff()
            if p.SAVE_BAG:
                p.recorder.stop_recording_srv()