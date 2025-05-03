import math

# Define leg dimensions
coxa_length = 0.01
femur_length = 0.06
tibia_length = 0.06
body_x = 0.18/2
body_y = 0.12/2
leg_angle = math.atan2(body_x, body_y)
r = 0.07
x = r * math.sin(leg_angle)
y = r * math.cos(leg_angle)
z = -0.06
print(f"leg_angle: {math.degrees(leg_angle)}")
print(f"r: {r}")
print(f"x: {x}")
print(f"y: {y}")
print(f"z: {z}")

# Define leg base positions
leg_base_positions = [
    [ -x , y , z],
    [  0 , r , z],
    [  x , y , z],
    [  x , -y , z],
    [  0 , -r , z],
    [ -x , -y , z]
]