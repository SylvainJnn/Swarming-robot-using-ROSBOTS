#a class that contains the parameter for a goal, also has a method to print the goal
class robot_goal:
    def __init__(self, new_x, new_y, new_z, new_roll, new_pitch, new_yaw, new_w):
        self.x = new_x
        self.y = new_y
        self.z = new_z

        self.roll = new_roll
        self.pitch = new_pitch
        self.yaw = new_yaw
        self.w = new_w

    def print_goal(self):
        print("Position")
        print(" x     ", self.x)
        print(" y     ", self.y)
        print(" z     ", self.z)

        print("\nOrientation")
        print(" roll  ", self.roll)
        print(" pitch ", self.pitch)
        print(" yaw   ", self.yaw)
        print(" w     ", self.w)
