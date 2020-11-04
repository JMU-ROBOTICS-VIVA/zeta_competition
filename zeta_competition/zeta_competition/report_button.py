"""Publishes an empty message to the "report_requested" topic
whenever the button is pressed.  Code borrowed from:

http://effbot.org/tkinterbook/

Author: Nathan Sprague
Version: 11/2020

"""

from tkinter import *
import rclpy
from std_msgs.msg import Empty

class ReportButton(object):
    """ Node that presents a tkInter button. """

    def __init__(self):
        """ Set up the button and the callback. """
        rclpy.init()
        self.node = rclpy.create_node('report_button')
        self.pub = self.node.create_publisher(Empty, 'report_requested', 10)
        master = Tk()
        master.title('Zeta Rescue Report')
        f = Frame(master, height=100, width=240)
        f.pack_propagate(0) # don't shrink
        f.pack()

        b = Button(f, text="CLICK FOR REPORT", command=self.callback)
        b.pack(fill=BOTH, expand=1)

        mainloop()


    def callback(self):
        self.node.get_logger().info("Report Requested.")
        self.pub.publish(Empty())

def main():
    ReportButton()
        
if __name__ == "__main__":
    main()
