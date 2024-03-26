import rospy
from geometry_msgs.msg import Twist
import getch

class ButtonMove():
    def __init__(self) -> None:
        self.vel = Twist()
        self.base_lin_value = 0.01
        self.base_ang_value = 0.05
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def up_lin_vel(self, value): #Forward
        self.vel.linear.x += value

    def down_lin_vel(self, value): #back
        self.vel.linear.x -= value

    def up_ang_vel(self, value): #left
        self.vel.angular.z += value

    def down_ang_vel(self, value): #right
        self.vel.angular.z -= value

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0

    def pub_vel(self):
        self.pub.publish(self.vel)

if __name__ == "__main__":
    rospy.init_node("button_move_node")
    bm = ButtonMove()

    while not rospy.is_shutdown():
        symbol = getch.getch()
        if(symbol == 'w'):
            bm.up_lin_vel(bm.base_lin_value)
        elif(symbol == 's'):
            bm.down_lin_vel(bm.base_lin_value)
        elif(symbol == 'a'):
            bm.up_ang_vel(bm.base_ang_value)
        elif(symbol == 'd'):
            bm.down_ang_vel(bm.base_ang_value)
        else:
            bm.stop()
        bm.pub_vel()
    rospy.spin()