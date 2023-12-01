import numpy as np

# def scanner_points(self):
xc = np.array([])
yc = np.array([])
zc = np.array([])
ca = np.array([])

x_start = 0
z_start = 0
x_end = 203
z_end = 203
y_start = 0
y_end = 203

step = 1
big_step = 10

degree = 45

flag = True  # this needs the value of the LDS read into it then converted based on the displacement to origin

rotations = 360 / degree

# align platform to 0 position

for h in range(0, int(rotations)):

    # move_servo_r(r_start)
    # move_servo_th(th_start)
    # move_servo_z(z_start)

    cur_axis = h * degree

    for k in range(y_start, y_end):  # Calculate next path between points
        y_pos = k

        for j in range(z_start, z_end):
            z_pos = j

            for i in range(x_start, x_end):
                x_pos = i

                if flag:
                    xc = np.append(xc, [x_pos])
                    # zc = np.append(yc, [z_pos])
                    # yc = np.append(zc, [y_pos]) # change this to the adjusted read in value from the sensor
                    # ca = np.append(ca, [cur_axis])
                if x_pos == x_end - 1:
                    pass
                    # move_servo_x(x_start)
                    # time.sleep(0.00001)  # Wait for motor to reset to beginning of loop
                else:
                    pass
                    # move_servo_x(x_pos + step)
                    # time.sleep(.00001)  # change this to just longer than feed rate
            if z_pos == z_end - 1:
                pass
                # move_servo_z(z_start)
                # time.sleep(0.00001)  # reset to beginning of loop
            else:
                pass
                # print('not z end')
                # move_servo_z(z_pos + step)
                # time.sleep(.00001)  # change this to just longer than feed rate
        # move_servo_y(y_pos + big_step)
        # time.sleep(1)
    # time.sleep(1)
    # rotate center platform by degree
# return the xc, yc, zc, and ca to point verification
