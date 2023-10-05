from Src.robot.arm.stepperhandler import StepperHandler

s = StepperHandler()

while 1:
    i = input("Enter X/Y/Z followed by POS: 'X12': ")
    F = 1500

    if i[0] == 'X' or i[0] == 'x':
        s.write_x(int(i[1:]), F)
    elif i[0] == 'Y' or i[0] == 'y':
        s.write_y(int(i[1:]), F)
    elif i[0] == 'Z' or i[0] == 'z':
        s.write_z(int(i[1:]), F)
    elif i[0] == 'A' or i[0] == 'a':
        s.write_a(int(i[1:]), F)
    elif i[0] == 'B' or i[0] == 'b':
        s.write_b(int(i[1:]), F * 2)
