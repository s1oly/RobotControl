import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# The file content is already in the variable file_content
# Parse the CSV data
data = []
current_trial = None

# Read the CSV file directly
file_path = "/home/shubaniyer/catkin_ws/src/rocky_scripts/src/data.csv"  # Update this with your actual file path
with open(file_path, 'r') as file:
    file_content = file.read()

for line in file_content.strip().split('\n'):
    if line.startswith('Trial'):
        current_trial = line.split()[1]
    elif line.startswith('time_elasped'):
        continue
    elif line.startswith('176065'):
        parts = line.split(',')
        if len(parts) >= 3:
            time_elapsed = float(parts[0])
            
            # Extract Robot_Angle Z value (third coordinate)
            robot_angle_str = parts[1].strip('[]')
            robot_angle_parts = robot_angle_str.split()
            if len(robot_angle_parts) >= 3:
                robot_z = float(robot_angle_parts[2])
                
                # Handle wrap-around: values > 180 should be treated as negative
                if robot_z > 0:
                    robot_z = robot_z - 360

                
                data.append({
                    'trial': current_trial,
                    'time_elapsed': time_elapsed,
                    'robot_angle_z': robot_z
                })

# Convert to DataFrame
df = pd.DataFrame(data)

# Create individual plots for each trial
trials = df['trial'].unique()

fig, axes = plt.subplots(len(trials), 1, figsize=(12, 4*len(trials)))
if len(trials) == 1:
    axes = [axes]

for i, trial in enumerate(trials):
    trial_data = df[df['trial'] == trial].copy()
    
    # Normalize time to start from 0 for each trial
    min_time = trial_data['time_elapsed'].min()
    trial_data['time_normalized'] = trial_data['time_elapsed'] - min_time
    
    axes[i].plot(trial_data['time_normalized'], trial_data['robot_angle_z'], 
                marker='o', markersize=3, linewidth=1)
    axes[i].set_title(f'Trial {trial} - Robot Angle Z over Time')
    axes[i].set_xlabel('Time (seconds)')
    axes[i].set_ylabel('Robot Angle Z (degrees)')
    axes[i].grid(True, alpha=0.3)
    
    # Add some statistics to the plot
    mean_val = trial_data['robot_angle_z'].mean()
    std_val = trial_data['robot_angle_z'].std()
    axes[i].axhline(y=-175, color='r', linestyle='--', alpha=0.7, 
                   label=f'Target: {175:.2f}°')
    axes[i].legend()

plt.tight_layout()
plt.show()

# Print some statistics for each trial
print("Trial Statistics:")
print("=" * 50)
for trial in trials:
    trial_data = df[df['trial'] == trial]
    print(f"Trial {trial}:")
    print(f"  Data points: {len(trial_data)}")
    print(f"  Mean Z-angle: {trial_data['robot_angle_z'].mean():.2f}°")
    print(f"  Std Dev: {trial_data['robot_angle_z'].std():.2f}°")
    print(f"  Min: {trial_data['robot_angle_z'].min():.2f}°")
    print(f"  Max: {trial_data['robot_angle_z'].max():.2f}°")
    print(f"  Range: {trial_data['robot_angle_z'].max() - trial_data['robot_angle_z'].min():.2f}°")
    print()