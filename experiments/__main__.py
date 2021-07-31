import xarm

try:
  arm = xarm.Controller('USB', debug=True)
except OSError:
  print("Failed to connect to LeArm")
  print("Checkout the docs: https://github.com/ccourson/xArmServoController/blob/main/Python/README.md")
  import sys
  sys.exit(1)

print('Battery voltage in volts:', arm.getBatteryVoltage())

servos = {
  "shoulder_pan": xarm.Servo(1, 1500),
  "shoulder_lift": xarm.Servo(2, 1500),
  "elbow": xarm.Servo(3, 1500),
  "wrist_flex": xarm.Servo(4, 1500),
  "wrist_roll": xarm.Servo(5, 1500),
  "grip_left": xarm.Servo(6, 1500),
}

def recv_joint_command(msg):
  changed = False
  for idx, name in enumerate(msg.name):

    # Check if we know how to control this joint
    servo = servos.get(name, None)
    if not servo:
      continue

    # Convert desired position to robot coordinates
    value = msg.position[idx]  # radians [-1.57, 1.57]
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
    arm.SetPosition(servos.values(), duration=1)


def main():
  # Zero out robot
  # FUTURE: read the current robot position from controller and update memory state
  arm.SetPosition(servos.values())

  print("TODO: subscribe to /joint_states and treat as joint commands!")


if __name__ == "__main__":
  main()
