"""
This is the implementation of the OGN node defined in OgnRobotPickNPlace.ogn
"""

import numpy as np
import carb
import os
from pathlib import Path
import traceback
from scipy.spatial.transform import Rotation as R

from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.core.articulations import Articulation
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from typing import Optional

from omni.isaac.core.scenes import Scene
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import quat_to_rot_matrix

SCALE_FACTOR = 0.05

class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(
        self,
        robot_articulation: Articulation,
        robot_description_path: str,
        urdf_path: str,
        end_effector_frame_name: Optional[str] = None,
    ) -> None:
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=robot_description_path,
            urdf_path=urdf_path,
        )
        if end_effector_frame_name is None:
            if "kinova_robot" in robot_description_path:
                end_effector_frame_name = "end_effector_link"
            else:
                end_effector_frame_name = "Link6"
        ArticulationKinematicsSolver.__init__(
            self, robot_articulation, self._kinematics, end_effector_frame_name
        )

def get_translate_from_prim(
    translation_from_source: np.ndarray,
    source_prim,
    target_info: tuple[np.ndarray, np.ndarray]
) -> np.ndarray:
    """
    특정 프림(source_prim)을 기준으로 한 translation_from_source가
    실제 월드 좌표계에서 어떻게 변환되는지 계산하여 반환합니다.
    """
    robot_pos, robot_orient = source_prim.get_world_pose()
    cur_target_object_pos, current_target_object_ori = target_info

    T_robot2world = np.eye(4, dtype=float)
    T_robot2world[0:3, 0:3] = quat_to_rot_matrix(robot_orient)
    T_robot2world[0:3, 3] = robot_pos

    T_obj2world = np.eye(4, dtype=float)
    T_obj2world[0:3, 0:3] = quat_to_rot_matrix(current_target_object_ori) * SCALE_FACTOR
    T_obj2world[0:3, 3] = cur_target_object_pos

    T_robot2obj = np.linalg.inv(T_robot2world) @ T_obj2world
    target_pos = np.pad(translation_from_source, (0, 1), constant_values=1)
    target_cube_relative_coordinates = (target_pos @ np.transpose(T_robot2obj))[0:3]

    return target_cube_relative_coordinates

class OgnRobotPickNPlaceInit(BaseResetNode):
    def __init__(self):
        self.initialized = None
        self.robot_prim_path = None
        self.ee_prim_path = None
        self.robot_name = None
        self.ee_name = None
        self.manipulator = None
        self.robot_controller = None
        self.robot_description_path = None
        self.urdf_path = None

        # waypoint 이동에 사용되는 변수들
        self.waypoints = None
        self.current_waypoint_idx = 0

        self.task = "Idle"
        self.timestamp = 0

        super().__init__(initialize=False)

    def initialize_scene(self):
        self.scene = Scene()
        self.manipulator = SingleManipulator(
            prim_path=self.robot_prim_path,
            name=self.robot_name,
            end_effector_prim_name=self.ee_name,
        )
        self.scene.add(self.manipulator)
        self.manipulator.initialize()
        self.robot = self.scene.get_object(self.robot_name)
        self.robot_controller = KinematicsSolver(
            robot_articulation=self.robot,
            robot_description_path=self.robot_description_path,
            urdf_path=self.urdf_path
        )
        self.articulation_controller = self.robot.get_articulation_controller()
        self.initialized = True
        self.timestamp = 0
        return

    def custom_reset(self):
        self.robot_controller = None


# ---------------------------
# 공통 웨이포인트 이동 로직을 함수로 정리
# ---------------------------
def move_along_waypoints(
    state: OgnRobotPickNPlaceInit,
    robot_xform_prim: XFormPrim,
    start_offset: np.ndarray,
    end_offset: np.ndarray,
    reference_info: tuple[np.ndarray, np.ndarray],
    orientation: np.ndarray,
    num_steps: int,
    next_task_name: str
) -> bool:
    """
    - state.waypoints가 None이면 초기화해서 start_offset에서 end_offset 방향으로 num_steps만큼 웨이포인트를 생성
    - 현재 waypoint로 IK를 구해 로봇을 이동
    - 모든 waypoint를 소화하면 다음 state.task를 next_task_name으로 바꾸고 True를 반환 (상태 전환 신호)
    - IK에 실패하면 False 반환
    """
    # 웨이포인트가 없다면 초기화
    if state.waypoints is None:
        start_point = np.round(
            get_translate_from_prim(start_offset, robot_xform_prim, reference_info),
            decimals=3
        )
        end_point = np.round(
            get_translate_from_prim(end_offset, robot_xform_prim, reference_info),
            decimals=3
        )
        state.waypoints = [
            start_point + (end_point - start_point) * i / (num_steps - 1)
            for i in range(num_steps)
        ]
        state.current_waypoint_idx = 0
        print(f"  -> Waypoints initialized: {state.waypoints}")

    # 현재 이동할 웨이포인트 추출
    cur_target = state.waypoints[state.current_waypoint_idx]

    # IK 계산
    actions, succ = state.robot_controller.compute_inverse_kinematics(
        target_position=np.array(cur_target),
        target_orientation=orientation
    )
    print(f"  -> IK success={succ}")

    # 성공 시 로봇 이동
    if succ:
        # ex) 매 3번 스텝마다만 apply_action 호출
        state.timestamp += 1
        if state.timestamp % 3 == 0:
            state.articulation_controller.apply_action(actions)
            state.current_waypoint_idx += 1

        print(f"  -> Reached waypoint! Next idx={state.current_waypoint_idx}")
        if state.current_waypoint_idx >= len(state.waypoints):
            # 모든 웨이포인트를 완료했으면 다음 태스크로 전환
            state.task = next_task_name
            state.waypoints = None
            state.current_waypoint_idx = 0
            return True
    else:
        carb.log_warn(f"IK did not converge to a solution for target {cur_target}")
        return False

    return False


class OgnRobotPickNPlace:
    @staticmethod
    def internal_state():
        return OgnRobotPickNPlaceInit()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        ee_down_orientation = np.array([0, 1, 0, 0])  # 엔드 이펙터가 아래로 향하도록 설정
        APART_TRANSLATE = 5
        CENTER_TRANSLATE = 3
        num_steps = 15

        try:
            if not state.initialized:
                # 로봇 초기화
                if db.inputs.robot_prim_path == "":
                    carb.log_warn("robot prim path is not set")
                    return False
                if "kinova" in db.inputs.robot_prim_path:
                    SELECTED_ROBOT = "kinova_robot"
                    state.robot_prim_path = db.inputs.robot_prim_path
                    state.ee_name = "end_effector_link"
                else:
                    SELECTED_ROBOT = "cr5_robot"
                    state.robot_prim_path = db.inputs.robot_prim_path
                    state.ee_name = "Link6"

                state.robot_name = SELECTED_ROBOT
                robot_cfg_path = Path(__file__).parent.parent / "robot" / SELECTED_ROBOT
                state.robot_description_path = os.path.join(robot_cfg_path, SELECTED_ROBOT + "_descriptor.yaml")
                state.urdf_path = os.path.join(robot_cfg_path, SELECTED_ROBOT + ".urdf")
                state.task = "Idle"  # 초기 상태
                state.initialize_scene()
                carb.log_info("Robot Initialized")

            current_target_object_ori = db.inputs.current_target_object_ori
            cur_target_object_pos = db.inputs.current_target_object_pos
            current_target_obj_info = (cur_target_object_pos, current_target_object_ori)

            object_target_ori = db.inputs.object_target_ori
            object_target_pos = db.inputs.object_target_pos
            placement_of_target_obj_info = (object_target_pos, object_target_ori)

            # 물체와 로봇 위치 가져오기
            robot_xform_prim = XFormPrim(state.robot_prim_path)

            # ========== 로그: 상태 표시 ==========
            print(f"\n[RobotPickNPlace] Current State: {state.task}")

            # 상태머신 로직
            if state.task == "Idle":
                # Idle 상태에서 바로 물체 위로 이동 상태로 전환
                print("  -> Transition to MoveAboveCube")
                db.outputs.gripper_grasp_command = False
                state.task = "MoveAboveCube"

            elif state.task == "MoveAboveCube":
                # 물체 위로 이동
                translation_from_source = np.array([0, 0, -APART_TRANSLATE])
                target_cube_relative_coordinates = get_translate_from_prim(
                    translation_from_source,
                    robot_xform_prim,
                    current_target_obj_info
                )
                actions, succ = state.robot_controller.compute_inverse_kinematics(
                    target_position=target_cube_relative_coordinates,
                    target_orientation=np.array(current_target_object_ori)
                )
                print(f"  -> IK Result: succ={succ}")
                if succ:
                    state.articulation_controller.apply_action(actions)
                    print("  -> Applying IK action, moving to ReachDown")
                else:
                    carb.log_warn("Failed to move above cube.")
                    print(traceback.format_exc())

                # 목표 지점(물체 위) 근접 여부 확인하여 다음 상태로
                ee_prim = XFormPrim(os.path.join(state.robot_prim_path,"robotiq_edited/Robotiq_2F_85/robotiq_2f_85_base_link"))
                cur_ee_position, _ = ee_prim.get_world_pose()
                distance_to_target_point = abs(
                    np.linalg.norm((cur_ee_position - cur_target_object_pos))
                    - np.linalg.norm(translation_from_source * SCALE_FACTOR)
                )
                print(f"  -> Distance to target point: {distance_to_target_point}")
                if distance_to_target_point < 0.02:
                    state.task = "ReachDown"

            elif state.task == "ReachDown":
                # APART_TRANSLATE -> CENTER_TRANSLATE 구간을 여러 웨이포인트로 이동
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -APART_TRANSLATE]),
                    end_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    reference_info=current_target_obj_info,
                    orientation=ee_down_orientation,
                    num_steps=num_steps,
                    next_task_name="Pick"
                )
                # 완료되면 Pick 상태로 전환

            elif state.task == "Pick":
                print("  -> In Pick state: closing gripper")
                db.outputs.gripper_grasp_command = True
                print("  -> Transition to PickUpObject")
                state.task = "PickUpObject"

            elif state.task == "PickUpObject":
                # CENTER_TRANSLATE -> APART_TRANSLATE (역방향)으로 여러 웨이포인트 이동
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    end_offset=np.array([0, 0, -APART_TRANSLATE]),
                    reference_info=current_target_obj_info,
                    orientation=ee_down_orientation,
                    num_steps=num_steps,
                    next_task_name="MoveToTargetAbove"
                )
                # 완료되면 MoveToTargetAbove 상태로 전환

            elif state.task == "MoveToTargetAbove":
                # 오브젝트를 옮길 위치 위로 이동
                translation_from_source = np.array([0, 0, -APART_TRANSLATE])
                target_cube_relative_coordinates = get_translate_from_prim(
                    translation_from_source,
                    robot_xform_prim,
                    placement_of_target_obj_info
                )
                actions, succ = state.robot_controller.compute_inverse_kinematics(
                    target_position=target_cube_relative_coordinates,
                    target_orientation=ee_down_orientation
                )
                print(f"  -> IK Result: succ={succ}")
                if succ:
                    state.articulation_controller.apply_action(actions)
                    print("  -> Applying IK action, moving to ReachDownTarget")
                else:
                    carb.log_warn("Failed to move above target cube.")
                    print(traceback.format_exc())

                ee_prim = XFormPrim(
                    os.path.join(
                        state.robot_prim_path, "robotiq_edited/Robotiq_2F_85/robotiq_2f_85_base_link"
                    )
                )
                cur_ee_position, _ = ee_prim.get_world_pose()
                distance_to_target_point = abs(
                    np.linalg.norm((cur_ee_position - object_target_pos))
                    - np.linalg.norm(translation_from_source * SCALE_FACTOR)
                )
                if distance_to_target_point < 1e-2:
                    state.task = "ReachDownTarget"

            elif state.task == "ReachDownTarget":
                # APART_TRANSLATE -> CENTER_TRANSLATE 구간을 여러 웨이포인트로 이동 (목적지)
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -APART_TRANSLATE]),
                    end_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    reference_info=placement_of_target_obj_info,
                    orientation=ee_down_orientation,
                    num_steps=num_steps,
                    next_task_name="Place"
                )
                # 완료되면 Place 상태로 전환

            elif state.task == "Place":
                print("  -> In Place state: opening gripper")
                db.outputs.gripper_grasp_command = False
                print("  -> Transition to MoveAwayFromTarget")
                state.task = "MoveAwayFromTarget"

            elif state.task == "MoveAwayFromTarget":
                # CENTER_TRANSLATE -> APART_TRANSLATE (역방향) 이동
                done = move_along_waypoints(
                    state,
                    robot_xform_prim,
                    start_offset=np.array([0, 0, -CENTER_TRANSLATE]),
                    end_offset=np.array([0, 0, -APART_TRANSLATE]),
                    reference_info=placement_of_target_obj_info,
                    orientation=ee_down_orientation,
                    num_steps=num_steps,
                    next_task_name="Done"
                )
                # 완료되면 Done 상태로 전환

        except Exception as error:
            db.log_warn(str(error))
            print(traceback.format_exc())
            return False

        return True
