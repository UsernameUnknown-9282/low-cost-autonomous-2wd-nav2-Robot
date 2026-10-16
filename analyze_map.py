from PIL import Image
import numpy as np

# Load pgm
img = Image.open('/mnt/c/Users/Jaya/Downloads/RC/house_full.pgm')
arr = np.array(img)
h, w = arr.shape
print(f"PGM size: {w}x{h}")

# In house_full.yaml:
# origin: [-10.0, -10.0, 0.0]
# resolution: 0.05
# occupied_thresh: 0.65
# free_thresh: 0.196
# negate: 0
# standard ROS map_server:
# p = pixel value (0 to 255)
# occ = (255 - p) / 255.0
# if occ > 0.65 -> occupied (p < 89)
# if occ < 0.196 -> free (p > 205)

origin_x = -10.0
origin_y = -10.0
res = 0.05

def world_to_map(x, y):
    mx = int((x - origin_x) / res)
    my = int((y - origin_y) / res)
    # Notice: In ROS occupancy grid, row 0 is bottom (y = origin_y), but in image row 0 is top!
    # standard map_server loads image with row 0 at the top, but converts so (0,0) in grid is bottom-left (h - 1 - my in image)!
    # Let's check both conventions:
    img_row_inverted = h - 1 - my
    img_row_normal = my
    return mx, my, img_row_inverted, img_row_normal

for x in [-2.5, -2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]:
    for y in [-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0]:
        mx, my, r_inv, r_norm = world_to_map(x, y)
        if 0 <= mx < w and 0 <= r_inv < h:
            val_inv = arr[r_inv, mx]
            if val_inv > 250: # clear white
                status = "FREE"
            elif val_inv < 50: # dark black
                status = "WALL"
            else:
                status = f"UNKNOWN({val_inv})"
            if status == "FREE":
                print(f"({x:4.1f}, {y:4.1f}) -> Grid({mx:3d}, {my:3d}) ImgRow({r_inv:3d}): {status}")
