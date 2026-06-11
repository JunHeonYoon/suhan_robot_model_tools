from pathlib import Path
from srmt2.planning_scene import PlanningScene
from srmt2.planner.rrt_connect import SuhanRRTConnect
from srmt2.kinematics import TRACIK
import numpy as np
np.set_printoptions(precision=3)
import time

script_dir = Path(__file__).resolve().parent
robot_dir = script_dir / "robots"

pc = PlanningScene(node_name="PlanningScene", 
                   arm_names=['fr3_arm'], 
                   hand_names=['fr3_hand'],
                   arm_dofs=[7], 
                   base_link="fr3_link0", 
                   urdf_file_path=str(robot_dir / "fr3.urdf"), 
                   srdf_file_path=str(robot_dir / "fr3.srdf"),)

tracIK = TRACIK(base_link="fr3_link0", 
                tip_link="fr3_hand_tcp", 
                urdf_file_path=str(robot_dir / "fr3.urdf"))

q_ready = np.array([0,0,0,-np.pi/2,0,np.pi/2,np.pi/4])
joint_upper_limit = tracIK.get_upper_bound()
joint_lower_limit = tracIK.get_lower_bound()

pc.display(q_ready)
pos_ready, quat_ready = tracIK.forward_kinematics(q_ready)
print("pos_ready: ", pos_ready)
print("quat_ready: ", quat_ready)

desired_pos = pos_ready + np.array([0.2, 0.0, -0.2])
desired_quat = quat_ready

for _ in range(100):
    print("Attempting to solve IK...")
    r, q_goal = tracIK.solve(desired_pos, desired_quat, q_ready)
    if r:
        print(q_goal)
        if pc.is_valid(q_goal):
            print("IK solution is valid")
            break
        else:
            print('fr3 is in collision')
            r = False
    else:
        print('fr3 IK failed')
        
if r:
    rrt_planner = SuhanRRTConnect(state_dim=7, lb=joint_lower_limit, ub=joint_upper_limit, validity_fn=pc.is_valid)
    rrt_planner.max_distance = 0.1
    rrt_planner.set_start(q_ready)
    rrt_planner.set_goal(q_goal)
    for _ in range(10):
        r, path = rrt_planner.solve()
        if r:
            # for q in path:
            #     pc.display(q)
            #     time.sleep(0.1)
            
            # ------------'
            traj_q, traj_qdot, traj_qddot, traj_time = pc.time_parameterize(path, max_velocity_scaling_factor=1.0, max_acceleration_scaling_factor=1.0)
            pc.display(traj_q[0])
            for i in range(1, len(traj_q)):
                time.sleep(traj_time[i] - traj_time[i-1])
                pc.display(traj_q[i])
            # ------------
            print("Path found")
            print("Times:", traj_time)
            break
        