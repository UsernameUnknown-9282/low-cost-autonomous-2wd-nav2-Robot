# Let's inspect the exact walls and free space near both trash cans
# Trash Can 1: (1.88, 1.91)
# Trash Can 2: (-4.69, 4.89)

from PIL import Image
import numpy as np

img = Image.open('/mnt/c/Users/Jaya/Downloads/RC/house_full.pgm')
arr = np.array(img)
h, w = arr.shape
origin_x, origin_y, res = -10.0, -10.0, 0.05

def check_region(cx, cy, name):
    print(f"\n=== Inspection around {name} at ({cx}, {cy}) ===")
    for dy in np.arange(0.5, -0.6, -0.2):
        row_str = f"y={cy+dy:4.2f}: "
        for dx in np.arange(-0.6, 0.7, 0.2):
            px, py = cx + dx, cy + dy
            mx = int((px - origin_x) / res)
            my = int((py - origin_y) / res)
            r_inv = h - 1 - my
            val = arr[r_inv, mx]
            if val > 240:
                row_str += " . " # free
            elif val < 50:
                row_str += " # " # wall/obstacle
            else:
                row_str += " ? "
        print(row_str)

check_region(1.88, 1.91, "Trash Can 1 (Living/Dining Room)")
check_region(-4.69, 4.89, "Trash Can 2 (Northwest Room)")
