import rospy
from std_msgs.msg import String

from hand_control import AbilityHandControl

class HandControlNode(object):

    def __init__(self):
        sleep_fn = lambda x: rospy.sleep(x)
        self.hand_control = AbilityHandControl(sleep_fn=sleep_fn)

        self.subscriber = rospy.Subscriber("chatter", String, self.callback)

    def callback(self, data):
        self.hand_control.close_interrupt_flag = True
        self.hand_control.open_interrupt_flag = True
        cmd = data.data.split("_")[0]
        fin = data.data.split("_")[1]
        # rospy.sleep(0.5)
        # split fin into list of fingers
        fins = [int(i) for i in fin]
        rospy.loginfo("cmd: %s, fin: %s", cmd, fins)
        if cmd == "open":
            # self.hand_control.close_interrupt_flag = True
            # self.hand_control.open_interrupt_flag = False
            self.hand_control.open_fingers_threaded(fins)
        elif cmd == "close":
            # self.hand_control.open_interrupt_flag = True
            # self.hand_control.close_interrupt_flag = False
            self.hand_control.close_fingers_threaded(fins)
        

if __name__ == "__main__":
    rospy.init_node('hand_control')
    hand_control_node = HandControlNode()
    rospy.spin()