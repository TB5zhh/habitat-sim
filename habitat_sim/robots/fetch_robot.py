import magnum as mn
import numpy as np
from habitat_sim.robots.mobile_manipulator import (MobileManipulator,
                                                   MobileManipulatorParams,
                                                   RobotCameraParams)


class FetchRobot(MobileManipulator):
    def __init__(self, urdf_path, sim, limit_robo_joints=True, fixed_base=True):
        fetch_params = MobileManipulatorParams(
            arm_joints=[2, 1, 3, 5, 11, 10, 4, 6, 13, 26, 39, 52],
            gripper_joints=[8, 9],
            wheel_joints=[13, 26, 39, 52],
            arm_init_params=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            gripper_init_params=[0.00, 0.00],
            ee_offset=mn.Vector3(0.1, -0.1, 0),
            ee_link=6,
            ee_constraint=np.array([[0.4, 1.2], [-0.7, 0.7], [0.25, 1.5]]),
            cameras={
                "robot_arm": RobotCameraParams(
                    #cam_offset_pos=mn.Vector3(-1.5, 1.6, -1.6),
                    #cam_look_at_pos=mn.Vector3(1.8, 0.0, -1.6),
                    cam_offset_pos=mn.Vector3(-0.15, 0.8, 0.0),
                    cam_look_at_pos=mn.Vector3(0.0, 0.0, 0.0),
                    attached_link_id=-1,
                ),
                "robot_head": RobotCameraParams(
                    # First person camera pos
                    cam_offset_pos=mn.Vector3(0.17, 0.075, -0.0325),
                    cam_look_at_pos=mn.Vector3(0.75, 0.075, -0.0325),
                    attached_link_id=-1,
                ),
                "robot_third": RobotCameraParams(
                    cam_offset_pos=mn.Vector3(-0.1, 0.1, -0.1),
                    cam_look_at_pos=mn.Vector3(1, 0.0, 0.75),
                    attached_link_id=-1,
                ),
            },
            gripper_closed_state=[0.0, 0.0],
            gripper_open_state=[1.57, 1.57],
            gripper_state_eps=0.001,
            arm_mtr_pos_gain=0.3,
            arm_mtr_vel_gain=0.3,
            arm_mtr_max_impulse=10.0,
            wheel_mtr_pos_gain=0.0,
            wheel_mtr_vel_gain=1.3,
            wheel_mtr_max_impulse=10.0,
            base_offset=mn.Vector3(0, 0, 0),
            base_link_names={
                "base_link",
                "r_wheel_link",
                "l_wheel_link",
                "r_wheel_link",
                "bellows_link",
                "bellows_link2",
                "estop_link",
                "laser_link",
                "torso_fixed_link",
            },
        )
        super().__init__(fetch_params, urdf_path, sim, limit_robo_joints, fixed_base)
        #self.back_joint_id = 8
        #self.head_rot_jid = 8
        #self.head_tilt_jid = 8

    def reconfigure(self) -> None:
        super().reconfigure()

        # NOTE: this is necessary to set locked head and back positions
        self.update()

    def reset(self) -> None:
        super().reset()

        # NOTE: this is necessary to set locked head and back positions
        self.update()

    @property
    def base_transformation(self):
        add_rot = mn.Matrix4.rotation(mn.Rad(-np.pi / 2), mn.Vector3(1.0, 0, 0))
        return self.sim_obj.transformation @ add_rot

    def update(self):
        super().update()
        # Fix the head.
        #self._set_joint_pos(self.head_rot_jid, 0)
        #self._set_motor_pos(self.head_rot_jid, 0)
        #self._set_joint_pos(self.head_tilt_jid, np.pi / 2)
        #self._set_motor_pos(self.head_tilt_jid, np.pi / 2)
        # Fix the back
        #fix_back_val = 0.15
        #self._set_joint_pos(self.back_joint_id, fix_back_val)
        #self._set_motor_pos(self.back_joint_id, fix_back_val)
