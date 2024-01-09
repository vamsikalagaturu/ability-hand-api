import rospy
from std_msgs.msg import String

from hand_control import AbilityHandControl

class HandControlNode(object):

    def __init__(self):
        self.hand_control = AbilityHandControl()

        self.subscriber = rospy.Subscriber("chatter", String, self.callback)

    def callback(self, data):
        cmd = data.data.split("_")[0]
        fin = data.data.split("_")[1]
        # split fin into list of fingers
        fins = [int(i) for i in fin]
        rospy.loginfo("cmd: %s, fin: %s", cmd, fins)
        if cmd == "open":
            self.hand_control.interrupt_flag = True
            self.hand_control.open_fingers_threaded(fins)
        elif cmd == "close":
            self.hand_control.interrupt_flag = True
            self.hand_control.close_fingers_threaded(fins)
        

if __name__ == "__main__":
    rospy.init_node('hand_control')
    hand_control_node = HandControlNode()
    rospy.spin()