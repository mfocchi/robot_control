import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
robotName = "climbingrobot2"
from base_controllers.utils.common_functions import getRobotModel, checkRosMaster
import base_controllers.params as conf
import numpy as np
import rospkg
from base_controllers.utils.math_tools import computeOrientationError
import pinocchio as pin
import sys


class ClosedLoopKinSolver():
    def __init__(self, robot_name="ur5"):
        self.robot_name = robot_name
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']

        self.robot = getRobotModel(self.robot_name, generate_urdf=True, xacro_path=rospkg.RosPack().get_path('climbingrobot_description') + '/urdf/' + self.robot_name + '.xacro')

        # example bounds; replace with your real limits
        self.q_min = np.array([-np.inf]*15)
        self.q_max = np.array([np.inf]*15)

    def fk_base_position(self, q):
        """
        Return base position as np.array([x, y, z]).
        Replace with your actual FK call.
        """
        qd = np.zeros_like(q)
        self.robot.computeAllTerms(q, qd)
        return self.robot.framePlacement(q, self.robot.model.getFrameId('base_link')).translation


    def fk_base_orient(self, q):
        """
        Return base orientation as quaternion [x, y, z, w].
        Replace with your actual FK call.
        """
        qd = np.zeros_like(q)
        self.robot.computeAllTerms(q, qd)
        R_world_base = self.robot.framePlacement(q, self.robot.model.getFrameId('base_link')).rotation

        return R_world_base

    def closure_frame_poses(self, q, link1="fake_link_end_of_right_rope", link2="base_link"):
        """
        Return the two closure frames involved in the fixed joint.
        Replace frame names with your actual closure frames.

        Returns:
            p_l, R_l, p_r, R_r
        where p_* are (3,), R_* are Rotation objects.
        """
        qd = np.zeros_like(q)
        self.robot.computeAllTerms(q, qd)

        p_l = self.robot.framePlacement(q, self.robot.model.getFrameId(link1)).translation
        p_r = self.robot.framePlacement(q, self.robot.model.getFrameId(link2)).translation

        R_l =  self.robot.framePlacement(q, self.robot.model.getFrameId(link1)).rotation
        R_r =  self.robot.framePlacement(q, self.robot.model.getFrameId(link2)).rotation


        return p_l, R_l, p_r, R_r

    def loop_residual(self, q):
        """
        6D closure residual:
        - 3 for position mismatch
        - 3 for orientation mismatch
        """
        p_l, R_l, p_r, R_r = self.closure_frame_poses(q)
        pos_err = p_l - p_r
        rot_err = computeOrientationError(R_l, R_r)
        return np.concatenate([pos_err, rot_err])

    def base_pose_residual(self, q, p_des, R_des):
        p_cur = self.fk_base_position(q)
        R_cur = self.fk_base_orient(q)

        pos_err = p_cur - p_des

        rot_err = computeOrientationError(R_cur, R_des)

        return pos_err, rot_err

    def residual(self, q, p_des, R_des, q_nom, q_prev):
        pos_err, base_rot_err = self.base_pose_residual(q, p_des, R_des)
        loop_err = self.loop_residual(q)
        nom_err = q - q_nom
        prev_err = q - q_prev

        # print("base pos:", np.linalg.norm(pos_err),
        #       "base rot:", np.linalg.norm(base_rot_err),
        #       "loop:", np.linalg.norm(loop_err))

        return np.concatenate([
            np.sqrt(self.wp) * pos_err,
            np.sqrt(self.wR) * base_rot_err,
            np.sqrt(self.wc) * loop_err,
            np.sqrt(self.wnom) * nom_err,
            np.sqrt(self.wprev) * prev_err,
        ])

    def solve(
            self,
            q0,
            p_des,
            R_des,
            q_nom=None,
            q_prev=None,
            wp=10.0,
            wR=10.0,
            wc=1000.0,
            wnom=1e-2,
            wprev=1e-2,
            max_nfev=300,
            verbose=True,
    ):

        self.wp = wp
        self.wR = wR
        self.wc = wc
        self.wnom = wnom
        self.wprev = wprev

        q0 = np.asarray(q0, dtype=float)
        p_des = np.asarray(p_des, dtype=float)


        if q_nom is None:
            q_nom = q0.copy()
        else:
            q_nom = np.asarray(q_nom, dtype=float)

        if q_prev is None:
            q_prev = q0.copy()
        else:
            q_prev = np.asarray(q_prev, dtype=float)

        res = least_squares(
            lambda q:self.residual(q,   p_des, R_des, q_nom, q_prev),
            q0,
            bounds=(self.q_min, self.q_max),
            xtol=1e-10,
            ftol=1e-10,
            gtol=1e-10,
            max_nfev=max_nfev,
            verbose=2 if verbose else 0
        )
        q_des = res.x
        pos_err, base_rot_err = self.base_pose_residual(q_des, p_des, R_des)
        loop_err = self.loop_residual(q_des)

        info = {
            "success": res.success,
            "status": res.status,
            "message": res.message,
            "cost": res.cost,
            "nfev": res.nfev,
            "q_des": q_des,
            "base_pos_error_norm": np.linalg.norm(pos_err),
            "base_rot_error_norm": np.linalg.norm(base_rot_err),
            "loop_error_norm": np.linalg.norm(loop_err),
            "base_pos_error": pos_err,
            "base_rot_error": base_rot_err,
            "loop_error": loop_err,
        }

        return q_des, info

    def print_solution(self, q_des, info):
        print("\n--------- q_des solution")
        for name, q in zip(self.joint_names, q_des):
            print(f"  {name:24s}: {q: .6f}")

        print("\n--------- residual summary")
        print(f"  success              : {info['success']}")
        print(f"  status               : {info['status']}")
        print(f"  cost                 : {info['cost']:.6e}")
        print(f"  nfev                 : {info['nfev']}")
        print(f"  base_pos_error_norm  : {info['base_pos_error_norm']:.6e}")
        print(f"  base_rot_error_norm  : {info['base_rot_error_norm']:.6e}")
        print(f"  loop_error_norm      : {info['loop_error_norm']:.6e}")
        print(f"  message              : {info['message']}")

    def computeJointVariables(self, p_des, R_des, q0, debug=False):
        q_nom = q0.copy()
        q_prev = q0.copy()
        q_des, info = self.solve(
            q0=q0,
            p_des=p_des,
            R_des=R_des,
            q_nom=q_nom,
            q_prev=q_prev,
            wp=10.0,
            wR=10.0,
            wc=1000.0,
            wnom=1e-2,
            wprev=1e-2,
            verbose=debug
        )
        if debug:
            self.print_solution(q_des, info)
        return q_des

if __name__ == '__main__':
    solver = ClosedLoopKinSolver(robot_name=robotName)
    q0 = np.zeros(15)
    p_des = np.array([0.72246, 0.5, 3.10526])
    R_des = np.eye(3)
    solver.computeJointVariables(p_des, R_des,q0, debug=True)