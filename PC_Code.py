

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import open3d as o3d
import numpy as np
from collections import defaultdict

# s = serial.Serial('COM6', 115200, timeout=1)  # Replace 'COM3' with your actual port if different

# print("Opening: " + s.name)

# # reset the buffers of the UART port to delete the remaining data in the buffers
# s.reset_output_buffer()
# s.reset_input_buffer()

# # wait for user's signal to start the program
# input("Press Enter to start communication...")
# # send the character 's' to MCU via UART
# # This will signal MCU to start the transmission


# done_check = 0
# out_list= []
# temp_str = ""
# temp_stat = False
# # recieve characters from UART of MCU
# while True:
#     x = s.read(1)             # read one 
#     dec = x.decode()
#     print(x.decode(), end="")
#     if dec == "[":
#         temp_stat = True
#     elif dec == "]":
#         out_list.append(temp_str)
#         temp_str = ""
#         temp_stat = False
#     if temp_stat == True:
#         temp_str += dec
#     if dec == "@":
#         break

# print(out_list)
# out_str = ' '.join(out_list)
# out_list = out_str.split("[")

# # for i in out_list

# with open('tof_radar.xyz', 'w') as f:
#     for i in out_list:
#         f.write(i)
#         f.write("\n")


# input_file = "tof_radar.xyz"

# with open(input_file, "r") as fin:
#     lines = fin.readlines()

# with open(input_file, "w") as fout:
#     for line in lines:
#         line = line.strip()
#         if not line or "Model ID" in line:
#             continue
#         parts = [p.strip() for p in line.split(",")]
#         if len(parts) == 3:
#             fout.write(f"{parts[0]} {parts[1]} {parts[2]}\n")


# Step 1: Read points from .xyz file
points = np.loadtxt("hall.xyz")

# Step 2: Group by Z value (rounded to nearest 0.5 cm to cluster together)
grouped_by_z = defaultdict(list)
for pt in points:
    x, y, z = pt
    z_key = round(z * 10) / 10  # adjust precision as needed
    grouped_by_z[z_key].append((x, y, z))

# Step 3: Build lines within each Z group
lines = []
line_points = []
point_offset = 0

for z_key, pts in grouped_by_z.items():
    # Sort by angle in XY plane
    pts = np.array(pts)
    angles = np.arctan2(pts[:,1], pts[:,0])
    sorted_indices = np.argsort(angles)
    pts = pts[sorted_indices]

    start_index = len(line_points)
    line_points.extend(pts)

    # Create line connections (i to i+1)
    for i in range(len(pts) - 1):
        lines.append([start_index + i, start_index + i + 1])
    # Optionally close the ring:
    if len(pts) > 2:
        lines.append([start_index + len(pts) - 1, start_index])

# Convert to Open3D LineSet
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(line_points)
line_set.lines = o3d.utility.Vector2iVector(lines)

# Load the original point cloud for visual comparison
pcd = o3d.io.read_point_cloud("hall.xyz")

# Step 4: Visualize together
o3d.visualization.draw_geometries([pcd, line_set])



# # Load a point cloud from file
# pcd = o3d.io.read_point_cloud("tof_radar.xyz")

# # Visualize it
# o3d.visualization.draw_geometries([pcd])
    
       
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"


#close the port
print("Closing: " + s.name)
s.close()
