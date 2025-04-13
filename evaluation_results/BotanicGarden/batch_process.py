import os
import numpy as np 
from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D
import shutil

# Define the input and output filenames
sequence_name = '1018_00'
sensor_type = 'stereo'
filename_prefix = '_'.join(['modules_vins', sensor_type, 'for', sequence_name, 'img10hz600p'])
openvins_estimated_surffix = 'estimated_traj'

if not os.path.exists(os.path.join(sequence_name, filename_prefix)):
    os.makedirs(os.path.join(sequence_name, filename_prefix))

testfolder_names = [f for f in os.listdir(os.path.join(sequence_name, filename_prefix))]
current_testfolder_name = '_'.join(['test', str(len(testfolder_names))])
if not os.path.exists(os.path.join(sequence_name, filename_prefix, current_testfolder_name)):
    os.makedirs(os.path.join(sequence_name, filename_prefix, current_testfolder_name))
if not os.path.exists(os.path.join(sequence_name, filename_prefix, current_testfolder_name, 'configs')):
    os.makedirs(os.path.join(sequence_name, filename_prefix, current_testfolder_name, 'configs'))    



# Open the input file for reading and the output file for writing
with open(os.path.join(sequence_name, '_'.join([filename_prefix, openvins_estimated_surffix+'.txt'])), 'r') as infile, open(os.path.join(sequence_name, filename_prefix, current_testfolder_name, '_'.join([filename_prefix, openvins_estimated_surffix, 'tum_format.txt'])), 'w') as outfile:
    # Process each line in the input file
    for line in infile:
        # Remove leading/trailing whitespace
        stripped_line = line.strip()
        # Skip empty lines
        if not stripped_line:
            continue
        # Split the line into tokens based on whitespace
        tokens = stripped_line.split()
        # Select the first eight tokens
        first_eight = tokens[:8]
        # Join the tokens back into a single string
        new_line = ' '.join(first_eight)
        # Write the new line to the output file
        outfile.write(new_line + '\n')

shutil.copy(os.path.join(sequence_name, '_'.join([filename_prefix, openvins_estimated_surffix+".txt"])), os.path.join(sequence_name, filename_prefix, current_testfolder_name, '_'.join([filename_prefix, openvins_estimated_surffix+'.txt'])))
shutil.copytree(os.path.join('../../configs', 'BotanicGarden'), os.path.join(sequence_name, filename_prefix, current_testfolder_name, 'configs', 'BotanicGarden'))



# because KeyFrame is in Xsens coordinates, so we need to transfer it to RGB0 coordinates
# VLP16 in RGB0 coordinates, T_rgb0_vlp16
sensor_coordinate_transform_matrix_for_camera_frame = np.array([
    [0.0238743541600432, -0.999707744440396, 0.00360642510766516, 0.138922870923538],  
    [-0.00736968896588375, -0.00378431903190059, -0.999965147452649, -0.177101909101325],  
    [0.999687515506770, 0.0238486947027063, -0.00745791352160211, -0.126685267545513],  
    [0.0,0.0,0.0,1.0]
])

# seems x in key frame of orbslam system is -z in BotanicGarden
# system_coordinate_transform_matrix_for_camera_frame = np.array([
#     [0., -1.,   0.,  0],  
#     [1., 0.,   0.,  0],  
#     [0., 0.,   1.,  0],  
#     [0., 0,    0,   1.0]
# ])

system_coordinate_transform_matrix_for_camera_frame = np.array([
    [1., 0.,   0.,  0],  
    [0., 1.,   0.,  0],  
    [0., 0.,   1.,  0],  
    [0., 0,    0,   1.0]
])

def invert_transformation_matrix(T):
    """
    Computes the inverse of a 4x4 homogeneous transformation matrix.
    
    Parameters:
        T (numpy.ndarray): 4x4 transformation matrix
    
    Returns:
        numpy.ndarray: 4x4 inverse transformation matrix
    """
    # Extract rotation (R) and translation (t)
    R = T[:3, :3]  # 3x3 rotation matrix
    t = T[:3, 3]   # 3x1 translation vector

    # Compute the inverse transformation
    R_inv = R.T  # Transpose of rotation matrix
    t_inv = -R_inv @ t  # Compute new translation

    # Construct the inverse transformation matrix
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv



traj = file_interface.read_tum_trajectory_file(open(os.path.join(sequence_name, filename_prefix, current_testfolder_name, '_'.join([filename_prefix, openvins_estimated_surffix, 'tum_format.txt']))))

transformed_poses = []
for pose in traj.poses_se3:

    # Tvi * Tib
    transformed_pose_matrix = system_coordinate_transform_matrix_for_camera_frame @ invert_transformation_matrix(sensor_coordinate_transform_matrix_for_camera_frame) @ pose

    # Append transformed pose as SE3 object
    transformed_poses.append(transformed_pose_matrix)

traj = PoseTrajectory3D(
    poses_se3=transformed_poses,
    timestamps=traj.timestamps
)

file_interface.write_tum_trajectory_file(os.path.join(sequence_name, filename_prefix, current_testfolder_name, '_'.join([filename_prefix, openvins_estimated_surffix, 'coordinate_aligned.txt'])), traj)