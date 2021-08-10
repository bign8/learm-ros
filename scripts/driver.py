#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import xarm

try:
    arm = xarm.Controller('USB', debug=True)
except OSError:
    print("Failed to connect to LeArm")
    print("Checkout the docs: https://github.com/ccourson/xArmServoController/blob/main/Python/README.md")
    import sys
    sys.exit(1)

print('Battery voltage in volts:', arm.getBatteryVoltage())

# Initialize with invalid value so first received coordinate gets sent to robot
# FUTURE: read the current robot position from controller and update memory state
servos = {
    "shoulder_pan": xarm.Servo(6, -1),
    "shoulder_lift": xarm.Servo(5, -1),
    "elbow": xarm.Servo(4, -1),
    "wrist_flex": xarm.Servo(3, -1),
    "wrist_roll": xarm.Servo(2, -1),
    "grip_right": xarm.Servo(1, -1),  # Flipped in URDF (open is close)
}

"""
Convert model inputs (radians) to servo PWM number (500-2500)
"""
def convert(value):# radians [-1.57, 1.57]
    value /= 1.57  # convert [-1, 1]
    value *= 1000  # convert [-1000, 1000]
    value += 1500  # convert [500, 2500]
    return int(value)  # truncate decimals so library operates correctly


def backward(value):# numberz [500, 2500]
    value -= 1500   # convert [-1000, 1000]
    value /= 1000   # convert [-1, 1]
    value = float(value) # decimals are okay
    value *= 1.57   # radians
    return value


class subscriber(object):
    def __init__(self, name):
        self.servo_id = servos.get(name).servo_id

    def __call__(self, float64):
        arm.setPosition(self.servoid, position=convert(float64), wait=False)


def main():
    # TODO: include robot serial code in topic
    rospy.init_node('driver', anonymous=True)

    # enable receiving external commands
    for name in servos:
        rospy.Subscriber(f"{name}_position_controller/command", Float64, subscriber(name))

    # outbound joint status
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        # read servo positions to construct live object
        xarm.getPositions(list(servos.values()))
        update = JointState(
            name=list(servos.keys()),
            position=[backward(s.position) for s in servos.values()],
        )
        print(update, flush=True)
        pub.publish(update)

        rate.sleep()
    # rospy.spin()


if __name__ == '__main__':
    main()
