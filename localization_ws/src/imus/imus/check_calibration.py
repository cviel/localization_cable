import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from IMUdriver import IMU

def rotation_matrix(yaw, pitch, roll):
    """
    Create a rotation matrix from yaw, pitch, and roll angles.
    Yaw is rotation around the Z axis (inverted).
    Pitch is rotation around the Y axis (inverted).
    Roll is rotation around the X axis.
    """
    # Convert angles to radians
    yaw = np.radians(-yaw)
    pitch = np.radians(-pitch)
    roll = np.radians(roll)
    
    # Rotation matrix around the Z axis (yaw)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # Rotation matrix around the Y axis (pitch)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Rotation matrix around the X axis (roll)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Combined rotation matrix
    R = Rz @ Ry @ Rx
    return R

def plot_arrows(ax, yaw, pitch, roll):
    # Clear the plot
    ax.cla()
    
    # Define the coordinate frame (three arrows for x, y, z axes)
    origin = np.array([0, 0, 0])
    x_axis = np.array([5, 0, 0])
    y_axis = np.array([0, 5, 0])
    z_axis = np.array([0, 0, 5])
    
    # Create the rotation matrix
    R = rotation_matrix(yaw, pitch, roll)
    
    # Apply the rotation matrix to each axis
    rotated_x_axis = x_axis @ R.T
    rotated_y_axis = y_axis @ R.T
    rotated_z_axis = z_axis @ R.T
    
    # Plot the arrows representing the coordinate frame
    ax.quiver(origin[0], origin[1], origin[2],
              rotated_x_axis[0], rotated_x_axis[1], rotated_x_axis[2],
              length=1.0, color='r', arrow_length_ratio=0.1, label='X axis')
    ax.quiver(origin[0], origin[1], origin[2],
              rotated_y_axis[0], rotated_y_axis[1], rotated_y_axis[2],
              length=1.0, color='g', arrow_length_ratio=0.1, label='Y axis')
    ax.quiver(origin[0], origin[1], origin[2],
              rotated_z_axis[0], rotated_z_axis[1], rotated_z_axis[2],
              length=1.0, color='b', arrow_length_ratio=0.1, label='Z axis')
    
    # Set the limits of the plot
    ax.set_xlim([-6, 6])
    ax.set_ylim([-6, 6])
    ax.set_zlim([-6, 6])
    
    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Set the title
    ax.set_title(f'Yaw: {yaw}°, Pitch: {pitch}°, Roll: {roll}°')

    # Pause to update the plot
    plt.pause(0.01)

# Initialize the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

imu = IMU(port='/dev/ttyUSB0', parameter_path='src/imus/cfg')

while True:
    imu.update_once()
    yaw, pitch, roll = imu.euler_angles
    
    plot_arrows(ax, yaw, pitch, roll)
    
    time.sleep(0.05)  # Update every 0.1 seconds

plt.show()
