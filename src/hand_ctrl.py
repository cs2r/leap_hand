#!/usr/bin/env python

import rospy, math, time
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16


class Leap_ctrl():
    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def get_palm (self, data):
        #print data
        w_v = self.map(data.z, 0.8, -0.4, 100, 200)
        self.w_v.publish(w_v)

        w_h = self.map(data.x, 0.9, -0.9, 100, 250)
        self.w_h.publish(w_h)

    def get_thumb (self, data):
        #print data
        thumb = self.map(data.x, -0.7, 0.2, 100, 300)
        self.thumb.publish(thumb)

        thumb_j = self.map(data.y, -0.2, 0.5, 100, 300)
        self.thumb_j.publish(thumb_j)

    def get_index (self, data):
        #print data
        index = self.map(data.y, -0.3, 1.1, 100, 300)
        self.index.publish(index)

    def get_middle (self, data):
        #print data
        middle = self.map(data.y, -0.3, 0.9, 100, 300)
        self.middle.publish(middle)

    def get_ring (self, data):
        #print data
        ring = self.map(data.y, -0.3, 0.9, 100, 300)
        self.ring.publish(ring)

    def get_pinky (self, data):
        #print data
        pinky = self.map(data.y, -0.3, 0.9, 100, 300)
        self.pinky.publish(pinky)


    def __init__(self):
        rospy.init_node("hand_ctrl")
        rospy.Subscriber("palm_direction", Vector3, self.get_palm)
        rospy.Subscriber("thumb_direction", Vector3, self.get_thumb)
        rospy.Subscriber("index_direction", Vector3, self.get_index)
        rospy.Subscriber("middle_direction", Vector3, self.get_middle)
        rospy.Subscriber("ring_direction", Vector3, self.get_ring)
        rospy.Subscriber("pinky_direction", Vector3, self.get_pinky)
        self.w_v = rospy.Publisher("servo/L5", Int16, queue_size=10)
        self.w_h = rospy.Publisher("servo/L6", Int16, queue_size=10)
        self.thumb = rospy.Publisher("servo/L3", Int16, queue_size=10)
        self.thumb_j = rospy.Publisher("servo/L7", Int16, queue_size=10)
        self.index = rospy.Publisher("servo/L2", Int16, queue_size=10)
        self.middle = rospy.Publisher("servo/L1", Int16, queue_size=10)
        self.ring = rospy.Publisher("servo/L9", Int16, queue_size=10)
        self.pinky = rospy.Publisher("servo/L8", Int16, queue_size=10)

        print "leap hand is Ready"
        rospy.spin()


if __name__ == "__main__":
    leap_ctrl = Leap_ctrl()
