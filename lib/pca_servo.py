#!/usr/bin/env python3
import time
import Adafruit_PCA9685

# Initialize the PCA9685 on address 0x40 and bus 0 (as in your original code)
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

# Set frequency to 60 Hz
pwm.set_pwm_freq(60)

# Servo pulse range
servo_min = 300
servo_max = 750

# Initialize previous angles
prev_angles = [None] * 4

# Mapping functions
def map_to_servo(angle):
    return int((angle / 90.0) * (servo_max - servo_min) + servo_min)

# Function to apply servo angles to 4 servos
def move_servos(angles):
    global prev_angles
    angles = [min(max(angle, 0), 90) for angle in angles[:4]]
    print("Moving to:", angles)

    for i, angle in enumerate(angles):
        if prev_angles[i] != angle:
            if i == 0:
                pwm.set_pwm(1, 0, map_to_servo(angle))
            elif i == 1:
                pwm.set_pwm(0, 0, map_to_servo(angle))
            elif i == 2:
                pwm.set_pwm(15, 0, map_to_servo(angle))
            elif i == 3:
                pwm.set_pwm(14, 0, map_to_servo(angle))
            prev_angles[i] = angle

if __name__ == '__main__':
    # Test loop: move all servos from 0 to 90 and back
    try:
        while True:
            for angle in range(0, 50, 1):  # 0 to 90
                move_servos([angle, angle, angle, angle])
                time.sleep(0.01)
            for angle in range(50, 0, -1):  # 90 to 0
                move_servos([angle, angle, angle, angle])
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping servos")
        for ch in [0, 1, 14, 15]:
            pwm.set_pwm(ch, 0, 0)
