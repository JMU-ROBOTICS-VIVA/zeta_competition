"""
Code to publish victim messages for the purposes of testing
competition code.
"""
import rclpy
import rclpy.node
import numpy as np
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError

from zeta_competition_interfaces.msg import Victim
from sensor_msgs.msg import Image
import time


class VictimPublisher(rclpy.node.Node):
    """ 
    """

    def __init__(self, image_file):
        super().__init__('victim_pub')
        image = cv2.imread(image_file, cv2.IMREAD_COLOR)
        victim_pub = self.create_publisher(Victim, '/victim', 10)

        cv_bridge = CvBridge()
        input("press enter to publish.")
        for vic_num in range(5):
            victim_msg = Victim()

            img_out = np.array(image) # make a copy.
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img_out,str(vic_num),
                        (10, img_out.shape[0] - 50), font,
                        4, (255,255,255), 2, cv2.LINE_AA)

            img_msg_out = cv_bridge.cv2_to_imgmsg(img_out, "bgr8")
            img_msg_out.header.stamp = self.get_clock().now().to_msg()
            img_msg_out.header.frame_id = 'map'
            victim_msg.image = img_msg_out
            victim_msg.id = vic_num
            victim_msg.point.header = img_msg_out.header
            victim_msg.point.point.x = np.random.random()
            victim_msg.point.point.y = np.random.random()
            victim_msg.point.point.z = np.random.random()
            victim_msg.description = "Victim #{}".format(vic_num)
            victim_pub.publish(victim_msg)
            time.sleep(1.0)

def main(image_file):
    rclpy.init()
    node = VictimPublisher(image_file)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
   main(sys.argv[1])
