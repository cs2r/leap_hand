#!/usr/bin/env python

import sys, time, os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from src.lib import Leap
import rospy, time
from geometry_msgs.msg import Vector3



class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"
        rospy.init_node("leap_motion_publisher")
        self.palm_dir_pub = rospy.Publisher('palm_direction', Vector3 , queue_size=10)
        self.thumb_dir_pub = rospy.Publisher('thumb_direction', Vector3, queue_size=10)
        self.index_dir_pub = rospy.Publisher('index_direction', Vector3, queue_size=10)
        self.middle_dir_pub = rospy.Publisher('middle_direction', Vector3, queue_size=10)
        self.ring_dir_pub = rospy.Publisher('ring_direction', Vector3, queue_size=10)
        self.pinky_dir_pub = rospy.Publisher('pinky_direction', Vector3, queue_size=10)

    def on_connect(self, controller):
        print "Connected"

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()
        for hand in frame.hands:

            if ~hand.is_left:
                normal = hand.palm_normal
                direction = hand.direction
                arm = hand.arm
                palm_dir = Vector3()
                palm_dir.x = direction.pitch - arm.direction[1]
                palm_dir.y = normal.roll
                palm_dir.z = direction.yaw - arm.direction[0]
                self.palm_dir_pub.publish(palm_dir)

                # Get fingers
                finger_dir = Vector3()
                for finger in hand.fingers:
                    if finger.type == 0:  #thumb
                        finger_dir.x = finger.bone(3).direction[0] - finger.bone(2).direction[0]
                        finger_dir.y = finger.bone(1).direction[1] - normal.roll
                        print finger_dir.y
                        self.thumb_dir_pub.publish(finger_dir)

                    elif finger.type == 1:  #index
                        dire = finger.bone(1).direction - finger.bone(0).direction
                        finger_dir.x = dire[0]
                        finger_dir.y = dire[1]
                        finger_dir.z = dire[2]
                        self.index_dir_pub.publish(finger_dir)

                    elif finger.type == 2:  #middle
                        dire = finger.bone(1).direction - finger.bone(0).direction
                        finger_dir.x = dire[0]
                        finger_dir.y = dire[1]
                        finger_dir.z = dire[2]
                        self.middle_dir_pub.publish(finger_dir)
                    elif finger.type == 3:  # ring
                        dire = finger.bone(1).direction - finger.bone(0).direction
                        finger_dir.x = dire[0]
                        finger_dir.y = dire[1]
                        finger_dir.z = dire[2]
                        self.ring_dir_pub.publish(finger_dir)
                    elif finger.type == 4:  # pinky
                        dire = finger.bone(1).direction - finger.bone(0).direction
                        finger_dir.x = dire[0]
                        finger_dir.y = dire[1]
                        finger_dir.z = dire[2]
                        self.pinky_dir_pub.publish(finger_dir)

                time.sleep(0.1)

        if not (frame.hands.is_empty and frame.gestures().is_empty):
            print "NO Hand on field of view"

    def state_string(self, state):
        if state == Leap.Gesture.STATE_START:
            return "STATE_START"

        if state == Leap.Gesture.STATE_UPDATE:
            return "STATE_UPDATE"

        if state == Leap.Gesture.STATE_STOP:
            return "STATE_STOP"

        if state == Leap.Gesture.STATE_INVALID:
            return "STATE_INVALID"

def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()
