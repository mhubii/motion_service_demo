import os

import kinpy
import numpy as np
import rclpy
import xacro
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from motion_service_demo_msgs.srv import Position


class MoveNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name=node_name)
        self.joint_trajectory_action_client_ = ActionClient(
            self,
            FollowJointTrajectory,
            "/position_trajectory_controller/follow_joint_trajectory",
        )
        while not self.joint_trajectory_action_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                f"Action server {self.joint_trajectory_action_client_._action_name} not available, waiting..."
            )
        self.get_logger().info(
            f"Action server {self.joint_trajectory_action_client_._action_name} available!"
        )

        root_link_name = "lbr_link_0"
        end_link_name = "lbr_link_ee"

        self.chain = kinpy.build_serial_chain_from_urdf(
            data=self.load_urdf_("med7"),
            end_link_name=end_link_name,
            root_link_name=root_link_name,
        )

        # creating a service for this is BAD style! This is just a hack for a demo!!
        self.motion_service = self.create_service(
            Position, "/relative_move", self.motion_service_callback
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 1
        )
        self.joint_state = None

    def joint_state_callback(self, joint_state: JointState) -> None:
        # sort
        sorted_names = self.chain.get_joint_parameter_names(exclude_fixed=True)
        q, dq, effort = (
            np.zeros(len(joint_state.name)),
            np.zeros(len(joint_state.name)),
            np.zeros(len(joint_state.name)),
        )
        for i, name in enumerate(sorted_names):
            q[i] = joint_state.position[joint_state.name.index(name)]
            dq[i] = joint_state.velocity[joint_state.name.index(name)]
            effort[i] = joint_state.effort[joint_state.name.index(name)]
        joint_state.position = q.tolist()
        joint_state.velocity = dq.tolist()
        joint_state.effort = effort.tolist()
        joint_state.name = sorted_names

        # assign
        self.joint_state = joint_state

    def load_urdf_(self, model: str = "med7") -> str:
        path = os.path.join(
            get_package_share_directory("lbr_description"),
            "urdf",
            model,
            f"{model}.urdf.xacro",
        )
        urdf = xacro.process(path)
        return urdf

    def motion_service_callback(
        self, request: Position.Request, response: Position.Response
    ) -> Position.Response:
        if not self.joint_state:
            response.message = "No joint state received yet!"
            response.success = False
            return response

        # sort joint states
        q0 = np.array(self.joint_state.position)

        # transform request to root frame
        base_tf = self.chain.forward_kinematics(q0)
        target_tf = kinpy.Transform(
            pos=[
                request.target_position.x,
                request.target_position.y,
                request.target_position.z,
            ],
        )

        target_tf = base_tf * target_tf

        # inverse kinematics given the current joint configuration and the desired end-effector pose
        qd = self.chain.inverse_kinematics(
            pose=target_tf,
            initial_state=q0,
        )

        self.move_to_configuration(qd.tolist())

        response.success = True
        return response

    def move_to_configuration(self, q: list, sec_from_start: int = 2) -> None:
        joint_trajectory_goal = FollowJointTrajectory.Goal()
        goal_sec_tolerance = 1
        joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance

        point = JointTrajectoryPoint()
        point.positions = q
        point.time_from_start.sec = sec_from_start

        joint_trajectory_goal.trajectory.joint_names = self.joint_state.name
        joint_trajectory_goal.trajectory.points.append(point)

        # send goal
        goal_future = self.joint_trajectory_action_client_.send_goal_async(
            joint_trajectory_goal
        )

        # ignore result


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MoveNode("move_node"))
    rclpy.shutdown()


main()
