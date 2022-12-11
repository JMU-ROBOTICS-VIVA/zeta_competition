"""
GUI Node for scoring Zeta competition.

Author: Nathan Sprague
"""

from tkinter import *

import rclpy
import rclpy.node
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

from sensor_msgs.msg import Image
from zeta_competition_interfaces.msg import Victim
from rclpy.duration import Duration
from std_msgs.msg import Empty


class ReportButton(rclpy.node.Node):
    """Node that presents a tkInter button."""

    def __init__(self, true_file):
        """Set up the buttons and the callbacks."""

        super().__init__('score_listener')

        group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.create_subscription(Victim, 'victim',
                                 self.victim_callback, 10,
                                 callback_group=group)
        self.timer = self.create_timer(.1, self.run, callback_group=group)

        self.image_pub = self.create_publisher(Image, 'victim_image', 10)
        self.victim_pub = self.create_publisher(Marker, 'victim_estimate', 10)
        self.truth_pub = self.create_publisher(MarkerArray, 'victims_true', 10)
        self.request_pub = self.create_publisher(Empty, 'report_requested', 10)

        self.true_victims = self.read_csv_file(true_file)

        self.victims_requested = False
        self.victims = {}
        self.next_victim_to_test = 0

    def run(self):
        """This method is a bit of a hack to create a Python node that has a
        traditional main loop instead of using a timer callback for
        periodic computation.  It will be called by a timer callback
        in its own thread, then just kill the timer and never return.

        """
        self.destroy_timer(self.timer)

        master = Tk()
        master.title('Zeta Rescue Scoring')
        f = Frame(master, height=50, width=240)
        f.pack_propagate(0)  # don't shrink
        f.pack()
        self.report_button = Button(f, text="CLICK FOR REPORT",
                                    command=self.tk_report_callback)
        self.report_button.pack(fill=BOTH, expand=1)

        self.next_button = Button(f, text="CHECK NEXT",
                                  command=self.tk_next_callback)
        self.next_button.config(state=DISABLED)
        self.next_button.pack(fill=BOTH, expand=1)

        self.text = Text(master, height=9, width=30)
        self.text.insert(END, "")
        self.text.pack()
        mainloop()

    def victim_callback(self, victim):
        """Store the victim info in the dictionary and update the GUI."""

        if not self.victims_requested:
            self.get_logger().info("VICTIM RECIEVED EARLY, IGNORING id: {}".format(victim.id))
        else:
            self.get_logger().info("VICTIM RECIEVED id: {}".format(victim.id))
            self.next_button.config(state=ACTIVE)

            self.victims[victim.id] = victim

            txt = "{} victim reports recieved.".format(len(self.victims))
            self.text.delete("1.0", END)
            self.text.insert(END, txt)

            txt = "Check {} of {}".format(self.next_victim_to_test + 1,
                                          len(self.victims))
            self.next_button['text'] = txt

    def read_csv_file(self, file_name):
        result = []
        f = open(file_name, 'r')
        for line in f:
            entries = line.strip().split(",")
            for i in range(len(entries)):
                try:
                    entries[i] = float(entries[i].strip())
                except ValueError:
                    pass

            result.append(tuple(entries))
        f.close()
        return result

    def tk_next_callback(self):
        keys = sorted(self.victims.keys())
        cur_key = keys[self.next_victim_to_test]
        cur_vic = self.victims[cur_key]

        x = cur_vic.point.point.x
        y = cur_vic.point.point.y

        if self.is_near_true_victim(x, y):
            color = (0., 1., 0.)
            txt = "victim {} is a match".format(self.next_victim_to_test + 1)
        else:
            color = (1., 0., 0.)
            txt = "victim {} is not a match".format(self.next_victim_to_test + 1)

        self.text.delete("1.0", END)
        self.text.insert(END, txt)

        marker = self.make_sphere_marker(x, y, .4,
                                         100, color, scale=.25)

        self.image_pub.publish(cur_vic.image)
        self.victim_pub.publish(marker)

        self.next_victim_to_test = ((self.next_victim_to_test + 1) %
                                    len(self.victims))
        txt = "Check {} of {}".format(self.next_victim_to_test + 1,
                                      len(self.victims))
        self.next_button['text'] = txt

    def tk_report_callback(self):
        true_markers = self.victim_markers(self.true_victims)
        self.truth_pub.publish(true_markers)

        self.text.delete("1.0", END)
        self.text.insert(END, "WAITING FOR VICTIM REPORTS...")
        self.victims_requested = True
        self.request_pub.publish(Empty())

    def is_near_true_victim(self, x, y):
        for victim in self.true_victims:
            dist = np.sqrt((victim[0] - x) ** 2 + (victim[1] - y) ** 2)
            if dist < .3:
                return True
        return False

    def victim_markers(self, victims, color=(0.0, 0.0, 1.0)):

        marker_array = MarkerArray()
        index = 0
        for victim in victims:
            marker_array.markers.append(self.make_sphere_marker(victim[0],
                                                                victim[1],
                                                                .3,
                                                                index, color))
            index += 1

        return marker_array

    def make_sphere_marker(self, x, y, z,
                           ident=0, color=(1., 0., 0.), scale=.25):
        """ Create a sphere marker message at the indicated position. """
        marker = Marker()
        # go back in time a bit...
        dur = Duration(nanoseconds=100000000)
        now = self.get_clock().now()
        stamp = now - dur
        marker.header.stamp = stamp.to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'spheres'
        marker.id = ident
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.0, nanoseconds=0.0).to_msg()  # forever
        return marker


def main():
    if len(sys.argv) < 2:
        print("Usage: zeta_scorer.py /path/to/true_locations.csv")
        return

    rclpy.init()
    node = ReportButton(sys.argv[1])
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
