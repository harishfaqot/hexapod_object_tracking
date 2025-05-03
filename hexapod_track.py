import os
import time
import math
import threading
import cv2

from lib.servo import *
from lib.hexapod_constant import *
from lib.hexapod_control import *
from tracking import *

start_time = time.time()

# Shared variables for tracking
rel_x = 0
rel_y = 0
obj_label = None
obj_area = -1
frame = None
tracking_lock = threading.Lock()
stop_thread = False

def dxl_pos(radians):
    """Map a value from 0 to 300 degrees (in radians) to 0-1023 for Dynamixel."""
    max_radians = math.radians(300)
    radians = max(0, min(radians, max_radians))
    return int((radians / max_radians) * 1023)

# Inverse kinematics control for each leg
def joint_control(joint_index, pos):
    target_x, target_y, target_z = pos
    if joint_index >= 10:
        target_x *= -1
        target_y *= -1
    coxa_angle, femur_angle, tibia_angle = inverse_kinematics(target_x, target_y, target_z)

    base_index = (joint_index // 3) * 3
    if joint_index == 1:
        coxa_servo = math.radians(180 + 15) - leg_angle - coxa_angle
        femur_servo = math.radians(180 + 30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        goal_positions[0:3] = map(dxl_pos, [coxa_servo, femur_servo, tibia_servo])

    if joint_index == 4:
        coxa_servo = math.radians(180 + 15) - coxa_angle
        femur_servo = math.radians(180 + 30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        goal_positions[3:6] = map(dxl_pos, [coxa_servo, femur_servo, tibia_servo])

    if joint_index == 7:
        coxa_servo = math.radians(180 + 15) + leg_angle - coxa_angle
        femur_servo = math.radians(180 + 30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        goal_positions[6:9] = map(dxl_pos, [coxa_servo, femur_servo, tibia_servo])

    if joint_index == 10:
        coxa_servo = math.radians(180 + 15) - leg_angle - coxa_angle
        femur_servo = math.radians(180 + 30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        goal_positions[9:12] = map(dxl_pos, [coxa_servo, femur_servo, tibia_servo])

    if joint_index == 13:
        coxa_servo = math.radians(180 + 15) - coxa_angle
        femur_servo = math.radians(180 + 30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        goal_positions[12:15] = map(dxl_pos, [coxa_servo, femur_servo, tibia_servo])

    if joint_index == 16:
        coxa_servo = math.radians(180 + 15) + leg_angle - coxa_angle
        femur_servo = math.radians(180 + 30) - femur_angle
        tibia_servo = -math.radians(15) + tibia_angle
        goal_positions[15:18] = map(dxl_pos, [coxa_servo, femur_servo, tibia_servo])

    goal_positions[18] = 530
    goal_positions[19] = 850
    # move_servos([0, 50, 60, 0]) # [arm_2, grip_2, arm_1, grip1]

# Tracking thread to run in background
def tracking_thread():
    global rel_x, rel_y, obj_label, obj_area, frame, stop_thread
    while not stop_thread:
        try:
            new_frame, new_rel_x, new_rel_y, new_label, new_area = track_object()
            with tracking_lock:
                frame = new_frame
                rel_x = new_rel_x
                rel_y = new_rel_y
                obj_label = new_label
                obj_area = new_area
        except Exception as e:
            print(f"[Tracking Thread] Error: {e}")
            break

# Start tracking thread
thread = threading.Thread(target=tracking_thread)
thread.start()

# Main walking loop
step_duration = 1
step_h = 0.05
phase = 2

try:
    while True:
        t_start = time.time()
        t = time.time() - start_time

        # Use shared tracking data
        with tracking_lock:
            current_frame = frame
            current_rel_x = rel_x
            current_label = obj_label
            current_area = obj_area

        vx = 0
        vrot = current_rel_x / 250
        print(f"vrot = {vrot:.2f} label = {current_label} area={current_area}")

        if abs(vrot)<= 0.1 and current_label == 'korban' and current_area>=3500:
            print("Taking Korban...")
            take_object()
            print("Success Korban...")
        elif abs(vrot)<= 0.1 and current_label == 'korban' and current_area<=3500:
            vx = 0.5

        if current_frame is not None:
            cv2.imshow("YOLOv4-Tiny - Tracker", current_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        body_movement = (vx * 0.07, 0 * 0.07, vrot * -0.07)
        body_position = (0, 0, 0)
        body_orientation = (0, 0, 0)

        leg_pos_body = body_kinematics(body_position, body_orientation)

        for leg_index in range(6):
            pos, _ = generate_movement(t, phase, step_duration, step_h, body_movement, leg_index)
            leg_base = leg_pos_body[leg_index]
            leg_pos = (
                leg_base[0] + pos[0],
                leg_base[1] + pos[1],
                leg_base[2] + pos[2]
            )
            joint_control(joint_index=leg_index * 3 + 1, pos=leg_pos)

        servo.write(servo_ids, goal_positions)
        time.sleep(1 / 240)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt: Ctrl+C Pressed")
except Exception as e:
    print(f"[Main Loop] Error: {e}")

# Cleanup
stop_thread = True
thread.join()
servo.disable_torque(servo_ids)
servo.close()
cv2.destroyAllWindows()
