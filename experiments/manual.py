import xarm
import time

try:
  arm = xarm.Controller('USB', debug=True)
except OSError:
  print("Failed to connect to LeArm")
  print("Checkout the docs: https://github.com/ccourson/xArmServoController/blob/main/Python/README.md")
  import sys
  sys.exit(1)

print('Battery voltage in volts:', arm.getBatteryVoltage())

lose = [(1, 1500)]
grab = [(1, 2200)]
upright = [(2, 1500), (3, 1500), (4, 1500), (5, 1500)]
pose1 = [(2, 1500), (3, 1000), (4, 1000), (5, 1970)]
pivot1 = [(6, 1500)]
pivot2 = [(6, 2000)]

def move(pose, duration=1000):
  arm.setPosition(pose, duration=duration, wait=True)
  time.sleep(1)

import code
code.interact(local=locals())
