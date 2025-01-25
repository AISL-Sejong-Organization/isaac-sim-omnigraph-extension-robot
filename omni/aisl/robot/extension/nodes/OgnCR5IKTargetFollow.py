"""
This is the implementation of the OGN node defined in OgnIKFollowTarget.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy as np
import carb

from omni.isaac.core_nodes import BaseResetNode

from omni.isaac.core.articulations import Articulation
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from typing import Optional

from omni.isaac.core.scenes import Scene
import omni.isaac.core.utils.prims as prims_utils
from omni.graph.core import Database
from omni.isaac.core.utils.rotations import quat_to_euler_angles

from omni.isaac.core.utils.transformations import get_translation_from_target
import omni.isaac.core.utils.xforms as xforms_utils


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(
        self,
        robot_articulation: Articulation,
        end_effector_frame_name: Optional[str] = None,
    ) -> None:
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=r"/home/hyeonsu/Documents/Kit/shared/exts/omni.aisl.robot.extension/omni/aisl/robot/extension/nodes/robot_descriptor.yaml",
            urdf_path=r"/home/hyeonsu/Documents/Kit/shared/exts/omni.aisl.robot.extension/omni/aisl/robot/extension/nodes/cr5_robot.urdf",
        )
        if end_effector_frame_name is None:
            end_effector_frame_name = "Link6"
            ArticulationKinematicsSolver.__init__(
                self, robot_articulation, self._kinematics, end_effector_frame_name
            )


class OgnIKFollowTargetInit(BaseResetNode):
    def __init__(self):
        self.initialized = None
        self.robot_prim_path = None
        self.tcp_prim_path = None
        self.robot_name = None
        self.tcp_name = None
        self.manipulator = None
        self.my_controller = None
        super().__init__(initialize=False)

    def initialize_scene(self):
        self.scene = Scene()
        self.manipulator = SingleManipulator(
            prim_path=self.robot_prim_path,
            name=self.robot_name,
            end_effector_prim_name=self.tcp_name,
        )
        self.scene.add(self.manipulator)
        self.manipulator.initialize()
        self.my_robot = self.scene.get_object(self.robot_name)
        self.my_controller = KinematicsSolver(self.my_robot)
        self.articulation_controller = self.my_robot.get_articulation_controller()
        self.initialized = True
        return

    def custom_reset(self):
        self.my_controller = None


class OgnCR5IKTargetFollow:

    @staticmethod
    def internal_state():
        return OgnIKFollowTargetInit()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        robot_origin = prims_utils.get_prim_at_path("/World/dobot_assembly")
        target_cube = prims_utils.get_prim_at_path("/World/Cube")

        # try:
        pos, quaternion_orientation = xforms_utils.get_world_pose("/World/Cube")
        # orientation = quat_to_euler_angles(ori)

        target_cube_relative_coordinates = get_translation_from_target(
            [0, 0, -3], target_cube, robot_origin
        )

        if not state.initialized:
            state.robot_prim_path = db.inputs.robot_prim_path
            state.tcp_prim_path = db.inputs.end_effector_prim_path
            state.robot_name = db.inputs.robot_name
            state.tcp_name = db.inputs.end_effector_name
            state.initialize_scene()

        actions, succ = state.my_controller.compute_inverse_kinematics(
            target_position=np.array(target_cube_relative_coordinates),
            target_orientation=np.array(quaternion_orientation),
        )

        if succ:
            state.articulation_controller.apply_action(actions)
        else:
            carb.log_warn(
                f"IK did not converge to a solution for target {pos}. No action is being taken."
            )
        # except Exception as error:
        #     db.log_warn(str(error))
        #     return False
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            print("release_instance")
            state = Database.per_instance_internal_state(node)
        except Exception:
            state = None
            pass
        if state is not None:
            print("state is not None reset")
            state.reset()
