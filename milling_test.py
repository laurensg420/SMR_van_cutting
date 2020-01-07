import urx
import time
import copy


radius_rect = 0.020 # distance in meters
delta_x = 0.100  # distance in meters
delta_z = 0.100  # distance in meters

rob = urx.Robot("192.168.0.1")  # Set IP address
rob.set_tcp((0, 0, 0.135, 0, 0, 0))  # Set Tool Center Point
rob.set_payload(0.5, (0, 0, 0))  # Set payload for joints
time.sleep(0.2)  # leave some time to robot to process the setup commands

pos_start = [0.3577933706709267,
             1.0550768551162266,
             0.5982047398613053,
             -0.5248783789245691,
             1.8076119399319381,
             2.010596940359794]

a, v = (0.05, 0.5)


def homing():  # Move tool to top right corner for home position
    while rob.get_digital_in(switch_Y):
        rob.movel(0, 0.001, 0)

    while not rob.get_digital_in(switch_X):
        rob.movel(0.001, 0, 0)

    while not rob.get_digital_in(switch_Z):
        rob.movel(0, 0, 0.001)


def move_radius(radius, corner_no, clockwise=True, current_pos=True):
    if corner_no == 0 or corner_no == 1:
        delta_x_r = radius
    else:
        delta_x_r = -radius

    if corner_no == 0 or corner_no == 3:
        delta_z_r = radius
    else:
        delta_z_r = -radius

    cur_pos_r = rob.getl()

    pose_t = copy.deepcopy(cur_pos_r)
    pose_t[0] += delta_x_r
    pose_t[2] += delta_z_r

    half_sqrt_2 = 0.7071067811865475
    pose_v = copy.deepcopy(cur_pos_r)
    # pose_v[0] += half_sqrt_2 * delta_x
    if not corner_no % 2:  # if 0 or 2
        pose_v[0] += (1 - half_sqrt_2) * delta_x_r
        pose_v[2] += half_sqrt_2 * delta_z_r
    else:  # if 1 or 3
        pose_v[0] += half_sqrt_2 * delta_x_r
        pose_v[2] += (1 - half_sqrt_2) * delta_z_r

    rob.movec(pose_v, pose_t, a/5, v/5)


def draw_circle():
    x, y, z, rx, ry, rz = (delta_x, 0, 0, 0, 0, 0)
    rob.movel((x, y, z, rx, ry, rz), a, v, relative=True)
    print("Current tool pose2 is: ", rob.getl())


def draw_rectangle():
    x, y, z, rx, ry, rz = (delta_x, 0, 0, 0, 0, 0)
    rob.movel((x, y, z, rx, ry, rz), a, v, relative=True)
    print("Current tool pose2 is: ", rob.getl())

    move_radius(radius_rect, 1)

    x, y, z, rx, ry, rz = (0, 0, -delta_z, 0, 0, 0)
    rob.movel((x, y, z, rx, ry, rz), a, v, relative=True)
    print("Current tool pose3 is: ", rob.getl())

    move_radius(radius_rect, 2)

    x, y, z, rx, ry, rz = (-delta_x, 0, 0, 0, 0, 0)
    rob.movel((x, y, z, rx, ry, rz), a, v, relative=True)
    print("Current tool pose4 is: ", rob.getl())

    move_radius(radius_rect, 3)

    x, y, z, rx, ry, rz = (0, 0, delta_z, 0, 0, 0)
    rob.movel((x, y, z, rx, ry, rz), a, v, relative=True)
    print("Current tool pose5 is: ", rob.getl())

    move_radius(radius_rect, 0)

    print("Current tool pose6 is: ", rob.getl())


print("Current tool pose is: ", rob.getl())
rob.movel(pos_start, a,v)

cur_pos = rob.getl()  # get current position of robot
pos_start[0] = cur_pos[0]  # replace x
pos_start[1] = cur_pos[1]  # replace y
pos_start[2] = cur_pos[2]  # replace z

x, y, z, rx, ry, rz = pos_start

rob.movel((x, y, z, rx, ry, rz), a, v)
print("Current tool pose1 is: ", rob.getl())

#
# x, y, z, rx, ry, rz = (0.5, 0.021, 0, 0, 0, 0)
# rob.movel((x, y, z, rx, ry, rz), a, v, relative=True)
# print("Current tool pose1 is: ", rob.getl())
# 404.40
# print("Current tool pose1 is: ", rob.getl())

# while True:
#   x, y, z, rx, ry, rz = pos3
# rob.movel((x, y, z, rx, ry, rz), a, v)
#
# print("Current tool pose3 is: ", rob.getl())
# x, y, z, rx, ry, rz = pos2
# rob.movel((x, y, z, rx, ry, rz), a, v)
#
# print("Current tool pose2 is: ", rob.getl())
# print("hi")
# rob.translate((0.1, 0, 0), a, v)  # Move tool and keep orientation
# rob.stopj(a)  # Stop movement with current acc.


# rob.movex()
# rob.movel((x, y, z, rx, ry, rz), a, v, wait=False)
# while True:
#     time.sleep(0.1)  # sleep first since the robot may not have processed the command yet
#     if rob.is_program_running():
#         break
#
# rob.movel((x, y, z, rx, ry, rz), a, v, wait=False)
# while rob.get_force() < 50:
#     time.sleep(0.01)
#     if not rob.is_program_running():
#         break
# rob.stopl()
#
# try:
#     rob.movel((0, 0, 0.1, 0, 0, 0), relative=True)
# except RobotError as ex:
#     print("Robot could not execute move (emergency stop for example), do something", ex)
