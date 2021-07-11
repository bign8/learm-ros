#! /usr/bin/env python3

print("Forward Kinematics testing")

"""
Theta is defined as the angle above the horizontal that the gripper is facing.
Theta_5 (shoulder) ==> == 0deg
Theta_4 (elbow) (servo mounted bakcwards) <== == 0deg
Theta_3 (wrist) (servo mounted backwards) <== == 0deg

  elbow  wrist
    |      |
    v      v

    X------X
   /        \
  /          \
 /            \
X  <-- Shoulder

      ^ 90 degrees
+180  |
  === X ===> 0 degrees
-180  |
      v -90 degrees
"""
theta_cases = [
  # theta, theta_5 (shoulder angle), theta_4 (elbow servo), theta_3 (wrist servo)
  (0, 0, 90, 90),
  (180, 0, 0, 0),
  (180, 90, 0, 90),
  (180, 90, 90, 0),
  (90, 0, 0, 90),
  (90, 0, 90, 0),
  (-90, 0, 90, 180),
  (-180, 0, 180, 180),
  (180, 180, 90, 90),
  (360, 180, 0, 0),
]

theta = lambda shoulder, elbow, wrist: shoulder - elbow - wrist + 180

print("Test Theta")
for expected, shoulder, elbow, wrist in theta_cases:
  name = f"{expected:4} == theta({shoulder:3}, {elbow:3}, {wrist:3})"
  assert expected == theta(shoulder, elbow, wrist), f"FAIL {name}"
  print(f"PASS {name}")
