import xarm

try:
  arm = xarm.Controller('USB', debug=True)
except OSError:
  print("Checkout the docs: https://github.com/ccourson/xArmServoController/blob/main/Python/README.md")

print('Battery voltage in volts:', arm.getBatteryVoltage())

servos = [xarm.Servo(i, 1500) for i in range(1, 7)]

def write(positions):
    for i, p in enumerate(positions):
        servos[i].position = int(p)
    arm.setPosition(servos, duration=1, wait=True)

def read():
    pass
