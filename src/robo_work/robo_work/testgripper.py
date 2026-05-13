#!/usr/bin/env python3
import time
from gripper import RG

g = RG('rg2', '192.168.1.1', 502)

print("Start-Breite:", g.get_width(), "mm")

# Auf 80 mm öffnen
g.move_gripper(width_val=800, force_val=400)
time.sleep(3)
print("Nach 80 mm:", g.get_width(), "mm")

# Auf 30 mm schließen
g.move_gripper(width_val=300, force_val=400)
time.sleep(3)
print("Nach 30 mm:", g.get_width(), "mm")

# Auf 60 mm
g.move_gripper(width_val=600, force_val=400)
time.sleep(3)
print("Nach 60 mm:", g.get_width(), "mm")

g.close_connection()