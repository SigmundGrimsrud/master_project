import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import pandas as pd

model_name = 'mppi'
odom_file = os.path.expanduser(f'~/master/waypoints_auto/{model_name}_odometry.csv')
df = pd.read_csv(odom_file, header=None, usecols=[4,5], names=['x', 'y'])

waypoints = [ # simulation points
        [-0.05, 1.0],
        [-0.05, 2.0],
        [-0.05, 3.0],
        [-0.05, 4.5],
        [-0.05, 6.0],
        [-0.05, 7.5],
        [-0.05, 9.0],
        [-0.05, 7.5],
        [-0.05, 6.0],
        [-0.05, 4.5],
        [-0.05, 3.0],
        [-0.05, 1.5],
        [-0.05, 0.0],
        # Up and down first row
        [-0.05, 0.0], # Rotate 90 degrees left
        [-3.0,  0.0],
        [-6.0,  0.0],
        # Up and down second row
        [-6.0, 0.0,], # Rotate forward
        [-6.0, 1.0,],
        [-6.0, 2.0,],
        [-5.9, 3.0,],
        [-5.9, 4.5,],
        [-5.9, 6.0,],
        [-5.9, 7.5,],
        [-5.9, 6.0,],
        [-5.9, 4.5,],
        [-5.9, 3.0,],
        [-6.0, 1.5,],
        [-6.0, 0.0,]
        ]
waypoints_df = pd.DataFrame(waypoints, columns=['x', 'y'])


# Load image
# image_path = os.path.expanduser('~/Documents/Master_materials/video_timelap_image/map.png')
# image = plt.imread(image_path)

# # Image positioning and scaling parameters
# img_left = -15.5     # Left boundary in plot coordinates
# img_bottom = -10   # Bottom boundary in plot coordinates
# img_width = 100  # Width in plot coordinates
# img_height = 100 # Height in plot coordinates
# scale = 0.28      # Scale factor (1.0 = original size)

image_path = os.path.expanduser('~/Documents/Master_materials/video_timelap_image/map_small_crop.png')
image = plt.imread(image_path)

# Image positioning and scaling parameters
img_left = -9.8     # Left boundary in plot coordinates
img_bottom = -0.5   # Bottom boundary in plot coordinates
img_width = 100  # Width in plot coordinates
img_height = 100 # Height in plot coordinates
scale = 0.1375      # Scale factor (1.0 = original size)

# Calculate extent based on parameters
img_right = img_left + img_width * scale
img_top = img_bottom + img_height * scale
extent = [img_left, img_right, img_bottom, img_top]

# Create plot
fig, ax = plt.subplots(figsize=(10, 10))

# Display rotated image with specified extent
ax.imshow(image, extent=extent, aspect='auto')

# Plot dataframe values on top
colors = np.arange(len(df))
ax.scatter(df['x'], df['y'], c=colors, s=10, cmap='viridis', norm=Normalize(vmin=0, vmax=len(df)))

ax.scatter(waypoints_df['x'], waypoints_df['y'], c='red', s=15, label='Waypoints', marker='x')

cbar = plt.colorbar(ax.collections[0], ax=ax, orientation='vertical')
cbar.ax.tick_params(labelsize=20)
cbar.ax.yaxis.label.set_size(20)
cbar.set_label('Time')
cbar.ax.tick_params(labelsize=20)
cbar.set_ticks([0, len(df)])
cbar.ax.set_yticklabels(['start', 'stop'])

# Configure plot
ax.tick_params(axis='both', which='major', labelsize=15)
ax.xaxis.label.set_size(20)
ax.yaxis.label.set_size(20)
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_title(f'base_link coordinates when using {model_name}', fontsize=20)
ax.legend(fontsize=20)
ax.grid(True)


plt.tight_layout()
plt.show()