import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import pandas as pd

model_name = 'mppi'
recording = '1'

# Load and rotate image
image_path = os.path.expanduser('~/Documents/Master_materials/video_timelap_image/map.png')
image = plt.imread(image_path)
# rotated_image = np.rot90(image, k=2)  # Rotate 180 degrees (2*90 degree rotations)

# Create a sample dataframe - replace with your actual data

df = pd.read_csv('odometry.csv', header=None, usecols=[4,5], names=['x', 'y'])


# Image positioning and scaling parameters
img_left = -15.65     # Left boundary in plot coordinates
img_bottom = -10   # Bottom boundary in plot coordinates
img_width = 100  # Width in plot coordinates
img_height = 100 # Height in plot coordinates
scale = 0.278      # Scale factor (1.0 = original size)

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
ax.set_title('base_link coordinates on map', fontsize=20)
ax.legend()
ax.grid(True)


plt.tight_layout()
plt.show()