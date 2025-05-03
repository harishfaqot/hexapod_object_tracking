import math
import numpy as np
from lib.hexapod_constant import *
        
def generate_movement(t, phase_shift, step_duration, step_height, body_movement, leg_index):
    vx, vy, vrot = body_movement
    phase_shift+=0.0001
    phase_shifts = [i * step_duration/phase_shift for i in range(6)]
    phase = phase_shifts[leg_index]
    p = 1/phase_shift
    
    # Add phase shift to create staggered leg movement
    cycle_time = (t + phase) % step_duration  # Ensure that the cycle repeats every step_duration
    progress = cycle_time / step_duration  # Normalize progress between 0 and 1

    # Debugging: Check progress and cycle time
    # print(f"leg: {leg_index}, t: {t}, phase: {phase}, cycle_time: {cycle_time}, progress: {progress}")

    # Calculate the offset based on the leg's rotational component
    angle = math.pi / 2 + math.atan2(leg_base_positions[leg_index][0], leg_base_positions[leg_index][1])
    offset_x = vrot * math.sin(angle)
    offset_y = vrot * math.cos(angle)

    # Trajectory movement based on progress:
    if progress < p:  # Lifting phase (legs are lifting and moving forward)
        z = step_height * math.sin(2 * math.pi * progress)
        x = (vx + offset_x) * (progress / p - 0.5)
        y = (vy + offset_y) * (progress / p - 0.5)
    else:  # Lowering phase (legs are lowering and moving backward)
        z = 0
        x = (vx + offset_x) * (0.5 - (progress - p) / (1-p))
        y = (vy + offset_y) * (0.5 - (progress - p) / (1-p))
    

    # If no movement is desired (e.g., stationary), set z to 0
    if vx == 0 and vy == 0 and vrot == 0:
        z = 0

    return [x, y, z], progress

# Define inverse kinematics function
def inverse_kinematics(x, y, z):
    coxa_angle = math.pi / 2 + math.atan2(x, y)
    r = math.sqrt(x**2 + y**2)
    d = math.sqrt(r**2 + z**2)

    if d > femur_length + tibia_length:
        d = femur_length + tibia_length

    a1 = math.atan2(z, r)
    A = math.acos((d**2 + femur_length**2 - tibia_length**2) / (2 * d * femur_length))
    femur_angle = math.pi / 2 - (A + a1)

    B = math.acos((femur_length**2 + tibia_length**2 - d**2) / (2 * femur_length * tibia_length))
    tibia_angle = math.pi - B

    return coxa_angle, femur_angle, tibia_angle

# Define body kinematics with rotation
def body_kinematics(body_position, body_orientation):
    x_trans, y_trans, z_trans = body_position
    r, p, y = body_orientation
    # Multiply by 2 because robot will move 10 degree if moved 20 degree
    x_trans /= 20
    y_trans /= 20
    z_trans /= 20
    roll = math.radians(r) * -2
    pitch = math.radians(p) * 2
    yaw = math.radians(y) * 2

    # Compute the rotation matrix for the given roll, pitch, and yaw
    c_r, s_r = math.cos(roll), math.sin(roll)
    c_p, s_p = math.cos(pitch), math.sin(pitch)
    c_y, s_y = math.cos(yaw), math.sin(yaw)

    # Rotation matrix combining roll, pitch, and yaw
    rotation_matrix = np.array([
        [c_y * c_p, c_y * s_p * s_r - s_y * c_r, c_y * s_p * c_r + s_y * s_r],
        [s_y * c_p, s_y * s_p * s_r + c_y * c_r, s_y * s_p * c_r - c_y * s_r],
        [-s_p, c_p * s_r, c_p * c_r]
    ])

    leg_positions = []

    for leg_base in leg_base_positions:
        # Apply rotation to the leg base position
        rotated_leg_base = np.dot(rotation_matrix, np.array(leg_base))

        # Apply translation to the rotated position
        leg_x = rotated_leg_base[0] + x_trans
        leg_y = rotated_leg_base[1] + y_trans
        leg_z = rotated_leg_base[2] + z_trans

        leg_positions.append((leg_x, leg_y, leg_z))

    return leg_positions