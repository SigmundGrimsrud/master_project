import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import pandas as pd

waypoints = [ # simulation points
        [0.0, 0.0],
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
        # Up and down first row
        [-0.05, 0.0], # Rotate 90 degrees left
        [-3.0,  0.0],
        # Up and down second row
        [-6.0, 0.0], # Rotate forward
        [-6.0, 1.0],
        [-6.0, 2.0],
        [-5.9, 3.0],
        [-5.9, 4.5],
        [-5.9, 6.0],
        [-5.9, 7.5],
        [-5.9, 6.0],
        [-5.9, 4.5],
        [-5.9, 3.0],
        [-6.0, 1.5],
        [-6.0, 0.0]
        ]
waypoints_df = pd.DataFrame(waypoints, columns=['x', 'y'])


inspection_route = [ # simulation points
    [ 0.0,  0.0, np.pi/2],
    [-0.05, 1.0, np.pi/2],
    [-0.05, 2.0, np.pi/2],
    [-0.05, 3.0, np.pi/2],
    [-0.05, 4.5, np.pi/2],
    [-0.05, 6.0, np.pi/2],
    [-0.05, 7.5, np.pi/2],
    [-0.05, 9.0, np.pi/2],
    [-0.05, 7.5, np.pi/2],
    [-0.05, 6.0, np.pi/2],
    [-0.05, 4.5, np.pi/2],
    [-0.05, 3.0, np.pi/2],
    [-0.05, 1.5, np.pi/2],
    [-0.05, 0.0, np.pi/2],
    # Up and down first row
    [-0.05, 0.0, 0.0], # Rotate 90 degrees left
    [-3.0,  0.0, 0.0], # Actual pi
    [-6.0,  0.0, 0.0],
    # Down and up second row
    [-6.0, 0.0, np.pi/2], # Rotate forward
    [-6.0, 1.0, np.pi/2],
    [-6.0, 2.0, np.pi/2],
    [-5.9, 3.0, np.pi/2],
    [-5.9, 4.5, np.pi/2],
    [-5.9, 6.0, np.pi/2],
    [-5.9, 7.5, np.pi/2],
    [-5.9, 6.0, np.pi/2],
    [-5.9, 4.5, np.pi/2],
    [-5.9, 3.0, np.pi/2],
    [-6.0, 1.5, np.pi/2],
    [-6.0, 0.0, np.pi/2],
    ]

route_df = pd.DataFrame(inspection_route, columns=['x', 'y', 'theta'])

# models = ['mppi', 'ppc', 'dwb']
# for model_name in models:
#     # model_name = 'mppi'
#     odom_file = os.path.expanduser(f'~/master/waypoints_auto/{model_name}_odometry.csv')
#     df = pd.read_csv(odom_file, header=None, usecols=[4,5], names=['x', 'y'])

#     image_path = os.path.expanduser('~/Documents/Master_materials/video_timelap_image/map_small_crop.png')
#     image = plt.imread(image_path)

#     # Image positioning and scaling parameters
#     img_left = -9.8     # Left boundary in plot coordinates
#     img_bottom = -0.5   # Bottom boundary in plot coordinates
#     img_width = 100  # Width in plot coordinates
#     img_height = 100 # Height in plot coordinates
#     scale = 0.1375      # Scale factor (1.0 = original size)

#     # Calculate extent based on parameters
#     img_right = img_left + img_width * scale
#     img_top = img_bottom + img_height * scale
#     extent = [img_left, img_right, img_bottom, img_top]

#     # Create plot
#     fig, ax = plt.subplots(figsize=(13, 10))

#     # Display rotated image with specified extent
#     ax.imshow(image, extent=extent, aspect='auto')

#     # Plot dataframe values on top
#     colors = np.arange(len(df))
#     ax.scatter(df['x'], df['y'], c=colors, s=15, cmap='viridis', norm=Normalize(vmin=0, vmax=len(df)))

#     ax.scatter(waypoints_df['x'], waypoints_df['y'], c='red', s=40, label='Waypoints', marker='x')

#     cbar = plt.colorbar(ax.collections[0], ax=ax, orientation='vertical')
#     cbar.ax.tick_params(labelsize=30)
#     cbar.ax.yaxis.label.set_size(40)
#     cbar.set_label('Time')
#     cbar.ax.tick_params(labelsize=30)
#     cbar.set_ticks([0, len(df)])
#     cbar.ax.set_yticklabels(['start', 'stop'])

#     # Configure plot
#     ax.tick_params(axis='both', which='major', labelsize=30)
#     ax.xaxis.label.set_size(40)
#     ax.yaxis.label.set_size(40)
#     ax.set_xlabel('X Position')
#     ax.set_ylabel('Y Position')
#     ax.set_title(f'base_link coordinates when using {model_name.upper()}', fontsize=40)
#     ax.legend(fontsize=40)
#     ax.grid(True)


#     plt.tight_layout()
#     # plt.show()
#     plt.savefig(os.path.expanduser(f'~/master/waypoints_auto/{model_name}_path.png'))

#%% Calculate error
# For each point in df, find the distance**2 to the line between the previous and the next waypoint

# waypoints_df
# df

# models = ['mppi', 'ppc', 'dwb']

# for model_name in models:
#     odom_file = os.path.expanduser(f'~/master/waypoints_auto/{model_name}_odometry.csv')
#     df = pd.read_csv(odom_file, header=None, usecols=[4, 5, 9, 10], names=['x', 'y', 'z', 'w'])
#     # Pose orientation, z, w = index 9, 10
#     # Twist heading in column BA = index 52
#     # df = pd.DataFrame({'x': [0.0, 0.02, 0.06, 0.04, 0.03, 0.01, -0.02, -0.03, 0.0, 0.01, -0.02, 0.0, 0.0, 0.02, 0.01, -0.01], 'y': [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.6, 1.7, 1.8, 1.9]})

#     closest_waypoint = 0
#     goal_tolerance = 0.25
#     mse = 0
#     ase = 0
#     distance_plot = []
#     headings = []
#     heading_error_plot = []
#     theta_goals = []
#     for x, y, z, w in zip(df['x'], df['y'], df['z'], df['w']):
        # # Find the closest waypoint
        # # distances = np.sqrt((waypoints_df['x'] - x)**2 + (waypoints_df['y'] - y)**2)
        # # closest_index = np.argmin(distances)
        # # closest_waypoint = waypoints_df.iloc[closest_index]
        # if closest_waypoint < len(waypoints_df) - 1:
        #     if np.sqrt((waypoints_df['x'].iloc[closest_waypoint + 1] - x)**2 + (waypoints_df['y'].iloc[closest_waypoint + 1] - y)**2) < goal_tolerance:
        #         closest_waypoint += 1
        
        # # Calculate the distance to the line segment between current and next waypoints
        # if closest_waypoint == len(waypoints_df) - 1:
        #     prev_waypoint = waypoints_df.iloc[-2]
        #     next_waypoint = waypoints_df.iloc[-1]
        # else:
        #     prev_waypoint = waypoints_df.iloc[closest_waypoint]
        #     next_waypoint = waypoints_df.iloc[closest_waypoint + 1]

        # # Calculate the distance to the line segment
        # line_vec = np.array([next_waypoint['x'] - prev_waypoint['x'], next_waypoint['y'] - prev_waypoint['y']])
        # point_vec = np.array([x - prev_waypoint['x'], y - prev_waypoint['y']])
        # line_length_squared = np.dot(line_vec, line_vec)
        # t = max(0, min(1, np.dot(point_vec, line_vec) / line_length_squared))
        # projection = prev_waypoint + t * line_vec
        
        # distance_squared = np.sum((np.array([x, y]) - projection)**2)

        # if len(distance_plot) > 0:
        #     if abs(np.sum(np.array([x, y]) - projection) - distance_plot[-1]) > 0.05:
        #         continue


        # # Find the closest waypoint
        # distances = np.sqrt((route_df['x'] - x)**2 + (route_df['y'] - y)**2)
        # closest_index = np.argmin(distances)
        # closest_waypoint = route_df.iloc[closest_index]

        # Check if the next waypoint has a different heading goal
        # if closest_index < len(route_df) - 1:
        #     next_waypoint = route_df.iloc[closest_index + 1]
        #     if np.isclose(closest_waypoint['x'], next_waypoint['x'], atol=1e-2) and \
        #         np.isclose(closest_waypoint['y'], next_waypoint['y'], atol=1e-2):
        #         # If position goal does not change, use the next waypoint's heading
        #         goal_theta = next_waypoint['theta']
        #     else:
        #         goal_theta = closest_waypoint['theta']
        # else:
        #     goal_theta = closest_waypoint['theta']


        # mse+= distance_squared / len(df)
        # ase+= np.sum(abs(np.array([x, y]) - projection)) / len(df)
        # distance_plot.append(np.sum(np.array([x, y]) - projection))
        # distance_plot.append(distance_squared)

        # line_vec = np.array([next_waypoint['x'] - prev_waypoint['x'], next_waypoint['y'] - prev_waypoint['y']])
        # point_vec = np.array([x - prev_waypoint['x'], y - prev_waypoint['y']])
        # line_length_squared = np.sqrt(np.dot(line_vec, line_vec))
        # distance = np.abs(np.cross(line_vec, point_vec)) / line_length_squared
        # distance_squared = distance**2

        
        # mse+= distance_squared
        # distance_plot.append(distance)
    
    # create rolling average
    # times_to_average = 3
    # range_to_average = 5
    # for _ in range(times_to_average):
    #     averageed_plot = []
    #     for i, val in enumerate(distance_plot):
    #         if i > range_to_average and i < len(distance_plot) - range_to_average:
    #             averageed_plot.append(np.mean(distance_plot[i-range_to_average:i+range_to_average]))
    #     distance_plot = averageed_plot
    
    # if model_name == 'ppc':
    #     model_name = "rpp"
   
    # plt.figure(figsize=(10, 10), constrained_layout=True)
    # plt.plot(range(len(distance_plot)), distance_plot)
    # plt.plot(range(len(df['x'])), [0]*len(df['x']), 'r--')
    # #plt.plot(range(len(df['x'])), df['x'], 'g--')
    # plt.title(f'{model_name} distance to line', fontsize=40)
    # plt.xlabel('Time step', fontsize=35)
    # plt.ylabel('Distance to line [m]', fontsize=35)
    # plt.xticks(fontsize=30)
    # plt.yticks(fontsize=30)
    # plt.show()

    # plt.figure(figsize=(10, 10), constrained_layout=True)
    # plt.plot(range(len(headings)), headings)
    # plt.plot(range(len(heading_error_plot)), heading_error_plot, 'r--')
    # plt.plot(range(len(theta_goals)), theta_goals, 'g--')
    # plt.title(f'{model_name} heading', fontsize=40)
    # plt.xlabel('Time step', fontsize=35)
    # plt.ylabel('Heading [rad]', fontsize=35)
    # plt.xticks(fontsize=30)
    # plt.yticks(fontsize=30)
    # plt.show()

    # file_save = os.path.expanduser(f'~/master/waypoints_auto/{model_name}_distance_to_path_2.png')
    # # plt.savefig(file_save)

    # print(f"{model_name}'s Squared Error: {mse}")
    # print(f"{model_name}'s Absolute Error: {ase}")
    # print(f"{model_name}'s max overshoot: {np.max(distance_plot[650:800])}")


#%%

models = ['mppi', 'ppc', 'dwb']

for model_name in models:
    odom_file = os.path.expanduser(f'~/master/waypoints_auto/{model_name}_odometry.csv')
    df = pd.read_csv(odom_file, header=None, usecols=[4, 5, 9, 10], names=['x', 'y', 'z', 'w'])
    # Pose orientation, z, w = index 9, 10
    # Twist heading in column BA = index 52
    # df = pd.DataFrame({'x': [0.0, 0.02, 0.06, 0.04, 0.03, 0.01, -0.02, -0.03, 0.0, 0.01, -0.02, 0.0, 0.0, 0.02, 0.01, -0.01], 'y': [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.6, 1.7, 1.8, 1.9]})

    closest_waypoint = 0
    goal_tolerance = 0.25
    mse = 0
    ase = 0
    distance_plot = []
    headings = []
    heading_error_plot = []
    theta_goals = []
    for x, y, z, w in zip(df['x'], df['y'], df['z'], df['w']):
        # # Find the closest waypoint
        # # distances = np.sqrt((waypoints_df['x'] - x)**2 + (waypoints_df['y'] - y)**2)
        # # closest_index = np.argmin(distances)
        # # closest_waypoint = waypoints_df.iloc[closest_index]
        if closest_waypoint < len(waypoints_df) - 1:
            # Check if waypoint reached
            if np.sqrt((waypoints_df['x'].iloc[closest_waypoint + 1] - x)**2 + (waypoints_df['y'].iloc[closest_waypoint + 1] - y)**2) < goal_tolerance:
                # Check if the next waypoint has a different heading goal
                # if route_df.iloc[closest_waypoint]['theta'] != route_df.iloc[closest_waypoint + 1]['theta']:
                #     if heading - route_df.iloc[closest_waypoint]['theta'] < goal_tolerance:
                #         closest_waypoint += 1
                closest_waypoint += 1
        
        # Calculate the distance to the line segment between current and next waypoints
        if route_df.iloc[closest_waypoint]['x'] == route_df.iloc[closest_waypoint + 1]['x'] \
            and route_df.iloc[closest_waypoint]['y'] == route_df.iloc[closest_waypoint + 1]['y']:
            goal_theta = route_df.iloc[closest_waypoint + 1]['theta']

        # elif route_df.iloc[closest_waypoint]['x'] == route_df.iloc[closest_waypoint - 1]['x'] \
        #     and route_df.iloc[closest_waypoint]['y'] == route_df.iloc[closest_waypoint - 1]['y']:
        #     goal_theta = route_df.iloc[closest_waypoint - 1]['theta']
            
        elif closest_waypoint == len(waypoints_df) - 1:
            prev_waypoint = waypoints_df.iloc[-2]
            next_waypoint = waypoints_df.iloc[-1]
            goal_theta = route_df.iloc[-1]['theta']
        else:
            prev_waypoint = waypoints_df.iloc[closest_waypoint]
            next_waypoint = waypoints_df.iloc[closest_waypoint + 1]
            goal_theta = route_df.iloc[closest_waypoint]['theta']
        
        if closest_waypoint == 15:
            goal_theta = np.pi/2
            # goal_theta = 0.0

        if goal_theta == 0.0:
            goal_theta = np.pi/2
        elif goal_theta == np.pi/2:
            goal_theta = 0.0
        
        def quaternion_to_euler(z, w):
            t3 = +2.0 * (w * z)
            t4 = +1.0 - 2.0 * (z * z)
            Z = np.atan2(t3, t4)
            return Z
        heading = quaternion_to_euler(z, w)
        
        # heading = 2 * np.atan2(z, w)
        # if goal_theta == np.pi/2:
        #     # # Calculate the heading from quaternion
        #     heading += np.pi/2
        # elif goal_theta == 0.0:
        #     heading = np.pi/2 - heading

        headings.append(heading)
        theta_goals.append(goal_theta)

        # Calculate heading error
        heading_error = heading - goal_theta
        # # Normalize the error to the range [-pi, pi]
        # heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi

        # Store the heading error for analysis
        heading_error_plot.append(heading_error)

    # print(f"z     = {df.iloc[0]['z']}")
    # print(f"w     = {df.iloc[0]['w']}")
    # print(f"heading = {headings[0]}")
    # print(f"goal_theta = {theta_goals[0]}")
    # print(f"goal_error = {heading_error_plot[0]}")
    # print("------------------------------------")

    row_entering = 680
    print(f"Maximum heading error in row: {np.max(heading_error_plot[row_entering:])*180/np.pi}")

    if model_name == 'ppc':
        model_name = "rpp"
    plt.figure(figsize=(12, 12), constrained_layout=True)
    plt.plot(range(len(headings)), headings)
    plt.plot(range(len(heading_error_plot)), heading_error_plot, 'r--')
    plt.plot(range(len(theta_goals)), theta_goals, 'g--')
    plt.axhline(y=0.05, color='k', linestyle='--')
    plt.axhline(y=-0.05, color='k', linestyle='--')
    plt.axvline(x=row_entering, color='k', linestyle='--')
    plt.title(f'{model_name} heading', fontsize=40)
    plt.xlabel('Time step', fontsize=35)
    plt.ylabel('Heading [rad]', fontsize=35)
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.legend(['Heading', 'Heading error', 'Goal heading', 'Max angle goal'], fontsize=20)
    plt.show()

    file_save = os.path.expanduser(f'~/master/waypoints_auto/{model_name}_angle.png')
    # plt.savefig(file_save)

# print(quaternion_to_euler(0, 1))
# print(quaternion_to_euler(0.707, 0.707))
# print(quaternion_to_euler(1, 0))
