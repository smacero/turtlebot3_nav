'''robot spawns in at 
sample 1
    x = -1.999464 
    y = -0.500002 
    z = 0.008510 
    roll = 0.000105 
    pitch = 0.006040
    yaw = 0.00

 sample 2
    x = -1.999942
    y = -0.500001
    z = 0.008528
    roll = 0.000098
    pitch = 0.005738
    yaw = 0.000000
 
final pose with square waypoints:
x = -1.985767
y = -0.448033
z = 0.008526
roll = 0.000062
pitch = 0.005737
yaw = -0.092938

Final pose with hash waypoints:
x = -1.980600
y = -0.432266
z = 0.008528
roll = 0.000051
pitch = 0.005737
yaw = -0.093012
'''
import numpy as np

def compute_pose_error(final, initial):
    """Compute error between two poses"""
    trans_error = np.sqrt(
        (final['x'] - initial['x'])**2 + 
        (final['y'] - initial['y'])**2
    )
    
    # Normalize yaw difference 
    yaw_diff = final['yaw'] - initial['yaw']
    while yaw_diff > np.pi: yaw_diff -= 2*np.pi
    while yaw_diff < -np.pi: yaw_diff += 2*np.pi
    
    return {
        'translation_error': trans_error,
        'yaw_error': np.abs(yaw_diff)
    }

initial_pose1 = {
    'x': -1.999464,
    'y': -0.500002,
    'yaw': 0.0
}
initial_pose2 = {
    'x': -1.999942,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose3 = {
    'x': -1.999941,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose4 = {
    'x': -1.999941,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose5 = {
    'x': -1.999942,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose6 = {
    'x': -1.999952,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose7 = {
    'x': -1.999942,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose8 = {
    'x': -1.999942,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose9 = {
    'x': -1.999942,
    'y': -0.500001,
    'yaw': 0.0
}
initial_pose10 = {
    'x': -1.999942,
    'y': -0.500001,
    'yaw': 0.0
}

initial_poses = [
    initial_pose1, initial_pose2, initial_pose3, initial_pose4, initial_pose5,
    initial_pose6, initial_pose7, initial_pose8, initial_pose9, initial_pose10
]

square_final = {
    'x': -1.985767,
    'y': -0.448033,
    'yaw': -0.092938
}
hash_final = {
    'x': -1.980600,
    'y': -0.432266,
    'yaw': -0.093012
}

def compute_average_pose(poses):
    """Compute average of multiple poses"""
    x_sum = sum(pose['x'] for pose in poses)
    y_sum = sum(pose['y'] for pose in poses)
    yaw_sum = sum(pose['yaw'] for pose in poses)
    n = len(poses)
    
    return {
        'x': x_sum / n,
        'y': y_sum / n,
        'yaw': yaw_sum / n
    }

def compute_pose_std(poses, average):
    n = len(poses)
    x_var = sum((pose['x'] - average['x'])**2 for pose in poses) / n
    y_var = sum((pose['y'] - average['y'])**2 for pose in poses) / n
    yaw_var = sum((pose['yaw'] - average['yaw'])**2 for pose in poses) / n
    
    return {
        'x_std': np.sqrt(x_var),
        'y_std': np.sqrt(y_var),
        'yaw_std': np.sqrt(yaw_var)
    }


# Compute average and standard deviation
initial_average = compute_average_pose(initial_poses)
initial_std = compute_pose_std(initial_poses, initial_average)

print("\nInitial Pose Statistics:")
print(f"Average position: ({initial_average['x']:.6f}, {initial_average['y']:.6f})")
print(f"Position std dev: ({initial_std['x_std']:.6f}, {initial_std['y_std']:.6f})")
print(f"Average yaw: {initial_average['yaw']:.6f}")
print(f"Yaw std dev: {initial_std['yaw_std']:.6f}")

square_error = compute_pose_error(square_final, initial_average)
hash_error = compute_pose_error(hash_final, initial_average)
print("\nSquare Path Error:")
print(f"Translation error: {square_error['translation_error']:.4f} m")
print(f"Yaw error: {square_error['yaw_error']:.4f} rad")

print("\nHash Path Error:")
print(f"Translation error: {hash_error['translation_error']:.4f} m")
print(f"Yaw error: {hash_error['yaw_error']:.4f} rad")

# Compare square vs hash final poses
path_comparison = compute_pose_error(hash_final, square_final)
print("\nSquare vs Hash Final Pose Difference:")
print(f"Translation difference: {path_comparison['translation_error']:.4f} m")
print(f"Yaw difference: {path_comparison['yaw_error']:.4f} rad")
