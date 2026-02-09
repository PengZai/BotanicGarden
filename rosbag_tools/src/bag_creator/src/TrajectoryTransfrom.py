from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D
import numpy as np

input_path = "/media/spiderman/zhipeng_8t1/datasets/PolyTunnel_haygrove_Nov2025/Easy-1-loop/trajectories/orb_slam3/orb_m.txt"
out_path = input_path+".afterTransform.txt"

traj = file_interface.read_tum_trajectory_file(input_path)


# T = np.array([
#     [1., 0,  0.,  0],  
#     [0., 1., 0.,  0],  
#     [0.,  0., 1.,  0],  
#     [0,  0,  0,  1.0]
# ])


T = np.array([
    [0.0521232345, -0.0073054379,  0.9986139389,  0],  
    [-0.9986040017, -0.0089493528,  0.0520572461,  0],  
    [0.0085566474, -0.9999332676, -0.0077617087, 0],  
    [0,  0,  0,  1.0]
])


transformed_poses = []



for pose in traj.poses_se3:

    transformed_pose_matrix = T @ pose
    transformed_poses.append(transformed_pose_matrix)


transformed_traj = PoseTrajectory3D(
    poses_se3=transformed_poses,
    timestamps=traj.timestamps
)

file_interface.write_tum_trajectory_file(out_path, transformed_traj)