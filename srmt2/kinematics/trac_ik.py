from suhan_robot_model_tools2_wrapper_cpp import TRACIKAdapter, vectors_to_isometry, isometry_to_vectors
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from srmt2.utils.ros_utils import ros_init


class TRACIK:
    """
    Python adapter around the C++ TRAC-IK wrapper.
    Resolves the URDF either from another node's parameter server
    or directly from a file, then instantiates `TRACIKAdapter`.

    Parameters
    ----------
    base_link : str
        Name of the base link for IK.
    tip_link : str
        Name of the tip link (end-effector) for IK.
    max_time : float, optional
        IK timeout (seconds). Default = 0.1
    precision : float, optional
        Desired solution tolerance. Default = 1e-5
    param_node_name : str, optional
        Remote node that holds `robot_description`. Default = 'rviz2'
    robot_description_param : str, optional
        Name of the parameter that stores the URDF. Default = 'robot_description'
    urdf_file_path : str | None, optional
        If provided, load URDF directly from this file instead of a ROS 2 parameter.
    """
    def __init__(
        self,
        base_link: str,
        tip_link: str,
        max_time: float = 0.1,
        precision: float = 1e-5,
        param_node_name: str = "rviz2",
        robot_description_param: str = "robot_description",
        urdf_file_path: str | None = None,
    ):
        ros_init()

        if urdf_file_path is None:
            urdf_xml = self._fetch_remote_urdf(param_node_name, robot_description_param)
        else:
            with open(urdf_file_path, "r", encoding="utf-8") as f:
                urdf_xml = f.read()

        self.tracik = TRACIKAdapter(base_link, tip_link, max_time, precision, urdf_xml)
    

    def _fetch_remote_urdf(self, target_node: str, param_name: str) -> str:
        """Blocking URDF retrieval. Raises RuntimeError on failure."""
        tmp_node = Node("tracik_param_client")
        cli = tmp_node.create_client(GetParameters, f"{target_node}/get_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            self._cleanup(tmp_node)
            raise RuntimeError(
                f"GetParameters service on '{target_node}' is unavailable."
            )

        req = GetParameters.Request()
        req.names = [param_name]

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(tmp_node, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self._cleanup(tmp_node)
            raise RuntimeError("Timed out waiting for parameter response.")

        result = future.result()
        if not result.values or result.values[0].type != 4:  # PARAMETER_STRING
            self._cleanup(tmp_node)
            raise RuntimeError(
                f"Parameter '{param_name}' is missing or not a string."
            )
        urdf_xml = result.values[0].string_value

        if not urdf_xml:
            self._cleanup(tmp_node)
            raise RuntimeError(
                f"Parameter '{param_name}' on node '{target_node}' is empty."
            )

        self._cleanup(tmp_node)
        return urdf_xml

    def _cleanup(self, node: Node):
        # global ros_initialized
        node.destroy_node()
        # if ros_initialized:
            # rclpy.shutdown()
    
    def solve(self, pos, quat, q_init):
        assert(self.tracik.get_num_joints() == len(q_init), 'q_init size mismatch')

        iso = vectors_to_isometry(pos, quat)
        q_res = np.zeros(self.tracik.get_num_joints())
        r = self.tracik.solve(q_init, iso, q_res)
        return r, q_res

    def forward_kinematics(self, q):
        pos_quat = isometry_to_vectors(self.tracik.forward_kinematics(q))
        pos = pos_quat.first
        quat = pos_quat.second
        return pos, quat

    def get_lower_bound(self):
        return self.tracik.get_lower_bound()

    def get_upper_bound(self):  
        return self.tracik.get_upper_bound()

    def get_num_joints(self):
        return self.tracik.get_num_joints()

    def is_valid(self, q):
        return self.tracik.is_valid(q)

    def get_jacobian_matrix(self, q):
        return self.tracik.get_jacobian_matrix(q)

    def set_bounds(self, lb, ub):
        self.tracik.set_bounds(lb, ub)

    def set_tolerance_bounds(self, tol):
        self.tracik.set_tolerance_bounds(tol)

    def set_solve_type(self, type):
        self.tracik.set_solve_type(type)