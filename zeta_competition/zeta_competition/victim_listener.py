#!/usr/bin/env python
"""This is a simple node that listens for messages on the /victim topic
and republishes the image componenet as an image and the location component
as a marker.

This node also creates a csv file containing all of the final victim
information that was received as well as saved version of the victim
images.

Subscribes to:
   /victim

Publishes to:
   /victim_image
   /victim_marker

Author: Nathan Sprague
Version:  11/4/2020

"""
import sys
import rclpy
import rclpy.node
import rclpy.duration
import cv2
from cv_bridge import CvBridge, CvBridgeError

from zeta_competition_interfaces.msg import Victim
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

def make_sphere_marker(point, header,
                       ident=0, color=(1.,0.,0.), scale=.25):
    """ Create a sphere marker message at the indicated position. """
    marker = Marker()
    marker.header = header
    marker.ns = 'spheres'
    marker.id = ident
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z
    marker.scale.x = marker.scale.y = marker.scale.z = scale    
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0
    marker.lifetime =  rclpy.duration.Duration(seconds=0.0, nanoseconds=0.0).to_msg()# forever
    return marker


class VictimListener(rclpy.node.Node):
    """
    """

    def __init__(self, file_prefix):
        super().__init__('victim_listener')
        self.create_subscription(Victim, 'victim',
                                 self.victim_callback, 10)

        self.image_pub = self.create_publisher(Image, 'victim_image', 10)
        self.marker_pub = self.create_publisher(Marker, 'victim_marker', 10)

        self.cv_bridge = CvBridge()

        self.file_prefix = file_prefix

        self.victims = {}

    def save_victims(self):

        self.get_logger().info("Saving victim data...")
        outfile = open(self.file_prefix + ".csv", 'w')
        for victim_id in self.victims:
            victim = self.victims[victim_id]
            cv2_img = self.cv_bridge.imgmsg_to_cv2(victim.image,
                                                   desired_encoding="bgr8")
            img_name = "{}{}.jpg".format(self.file_prefix, victim_id)
            cv2.imwrite(img_name, cv2_img)
            line = '{}, {}, "{}", "{}"\n'.format(victim.point.point.x,
                                                 victim.point.point.y,
                                                 img_name,
                                                 victim.description)
            outfile.write(line)
        outfile.close()
        self.get_logger().info("Done.")

    def victim_callback(self, victim):
        """ Republish the victim message and store the victim info
        in the dictionary. """
        
        self.victims[victim.id] = victim
        self.image_pub.publish(victim.image)
        marker = make_sphere_marker(victim.point.point, victim.point.header,
                                    victim.id, (1., .65, 0.0))
        self.marker_pub.publish(marker)
        self.save_victims()


def main(prefix):
    rclpy.init()
    node = VictimListener(prefix)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Provide a prefix for the file..." + sys.argv[2])
    else:
        main(sys.argv[1])
