import aruco
from plot_3d_sympy import DigitalWorldHandler, plt
import sympy
import arduino_serial
import math

if __name__ == '__main__':

    # step 1
    # aruco.connect()
    # rvec, tvec = aruco.detect_marker(True)
    # print(rvec, tvec)
    world = DigitalWorldHandler()


    # arduino = arduino_serial.Arduino("COM8")
    rvec = [[-2.6604835],
            [-1.18965147],
            [0.27949938]]
    # rvec = [[-3.14],
    #         [0],
    #         [0]]

    tvec = [[1.887386],
            [-10.52237056],
            [113.97760565]]
    import numpy as np
    rvec = np.array(rvec).flatten()
    tvec = np.array(tvec).flatten()
    tvec = tvec/100
    # step 2 - move to aruco location with offset of 0.2 meters
    for i in range(0, 200,5):
        world.set_aruco_marker( rvec, tvec )
        world.move_to_marker()
        world.move_tcp((0, 0, -0.70))
        # print("Move tcp to marker and 0.70 back")
        world.plot(0.2)

        world.world.rotate_z_tcp(i/100*sympy.pi)
        # print("Rotate z-tcp")
        world.plot(0.2)
        a = world.world.get_angle_xy_plane_tcp()
        # print("Rotate align:{}".format(world.world.get_angle_xy_plane_tcp()))
        print('align')
        world.world.align_tcp_to_robot()
        print("{},{},{},{}".format(i/100, math.degrees(i/100*math.pi), math.degrees(a), math.degrees(world.world.get_angle_xy_plane_tcp())))
        world.plot(5)

        pose = world.getl()
    # rob.movel(pose, a,v)
    plt.pause(1000)