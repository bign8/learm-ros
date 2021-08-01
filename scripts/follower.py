#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
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

def recv_sensorMsgs_jointState(jointState):
    changed = False
    for idx, name in enumerate(jointState.name):

        # Check if we know how to control this joint
        servo = servos.get(name, None)
        if not servo:
            continue

        # Convert desired position to robot coordinates
        value = jointState.position[idx]  # radians [-1.57, 1.57]
        value /= 1.57  # convert [-1, 1]
        value *= 1000  # convert [-1000, 1000]
        value += 1500  # convert [500, 2500]
        value = int(value)  # truncate decimals so library operates correctly

        # Update the desired position in memory
        if servo.position != value:
            servo.position = value
            changed = True

    # If anything updated, notify the hardware
    if changed:
        print("Sending: ", {n: s.position for n, s in servos.items()})
        arm.setPosition(list(servos.values()), duration=1)

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", data.name, data.position)


def main():
    # TODO: include robot serial code in topic
    rospy.init_node('follower', anonymous=True)
    rospy.Subscriber("joint_states", JointState, recv_sensorMsgs_jointState)
    rospy.spin()


if __name__ == '__main__':
    main()
