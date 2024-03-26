import rospy
from geometry_msgs.msg import Twist

class SimpleMove():
    def __init__(self) -> None:
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    def move(self) -> None:
        vel = Twist()
        vel.linear.x = 0.1
        vel.angular.z = 0.1
        self.pub.publish(vel)

if __name__ == "__main__":
    rospy.init_node("simple_move_node")
    sm = SimpleMove()

    while not rospy.is_shutdown():
        sm.move()
    rospy.spin()