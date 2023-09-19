'''

    Object profiles is a solution to a problem with not being able to follow the same execution flow for
    probing a complex object.

    Rotational symmetric objects can be processed like such:
        - slice upon the z-axis
        - move probe to top most z-axis point, closest to the robot
        - revolve the platform, since the object is rotationally symmetirc, it will hit all points on Z-axis.
        - move down to next slice until some arbitrary z-height to avoid collision

    Rectangular objects can be processed like such:
        - slice and group on probe normals to group faces
        - sweep across a face
        - move probe back, rotate objetc
        - repeat

    Other objects ???
        - Would be cool to have an system for users to extend functionality with a script..?
'''

class ObjectProfile():

    def __init__(self, simulation, robot, point_cloud):
        pass

class RotationalSymmetric(ObjectProfile):

    def __init__(self):
        pass

class RectangularPrisms(ObjectProfile):

    def __init__(self):
        pass