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
    elif len(line) > 0:
        parts = line.split(',')
        if len(parts) >= 3:
            time_elapsed = float(parts[0])
            
            # Extract Robot_Angle Z value (third coordinate)
            robot_angle_str = parts[1].strip('[]')
            robot_angle_parts = robot_angle_str.split()
            if len(robot_angle_parts) >= 3:
                robot_z = float(robot_angle_parts[1])
                
                # Handle wrap-around: values > 180 should be treated as negative

                
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

# Store steady state errors for later use
steady_state_errors = {}

for i, trial in enumerate(trials):
    trial_data = df[df['trial'] == trial].copy()
    
    # Normalize time to start from 0 for each trial
    min_time = trial_data['time_elapsed'].min()
    trial_data['time_normalized'] = trial_data['time_elapsed'] - min_time
    
    # Calculate steady state error (average of last 5 points minus target)
    last_5_points = trial_data['robot_angle_z'].tail(5)
    avg_last_5 = last_5_points.mean()
    target = 5  # Since your data shows negative values for angles > 180
    steady_state_error = abs(avg_last_5 - target)
    steady_state_errors[trial] = steady_state_error
    
    axes[i].plot(trial_data['time_normalized'], trial_data['robot_angle_z'], 
                marker='o', markersize=3, linewidth=1)
    axes[i].set_title(f'Trial {trial} - 5 Degree 90o Rotation(Steady State Error: {steady_state_error:.2f}°)')
    axes[i].set_xlabel('Time (seconds)')
    axes[i].set_ylabel('Robot Angle Z (degrees)')
    axes[i].grid(True, alpha=0.3)
    
    # Add target line and steady state error annotation
    axes[i].axhline(y=target, color='r', linestyle='--', alpha=0.7, 
                   label=f'Target: {target}°')
    
    # Highlight the last 5 points used for steady state calculation
    last_5_times = trial_data['time_normalized'].tail(5)
    axes[i].scatter(last_5_times, last_5_points, color='red', s=30, zorder=5, 
                   label='Last 5 points (steady state)')
    
    # Add text annotation with steady state error
    axes[i].text(0.02, 0.98, f'Steady State Error: {steady_state_error:.2f}°', 
                transform=axes[i].transAxes, verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    axes[i].legend()

plt.tight_layout(pad=10.0)
plt.show()

# Create combined plot with all 5 trials
plt.figure(figsize=(12, 8))
colors = ['blue', 'green', 'orange', 'purple', 'brown']  # Colors for each trial

for i, trial in enumerate(trials):
    trial_data = df[df['trial'] == trial].copy()
    
    # Normalize time to start from 0 for each trial
    min_time = trial_data['time_elapsed'].min()
    trial_data['time_normalized'] = trial_data['time_elapsed'] - min_time
    
    # Get steady state error for this trial
    sse = steady_state_errors[trial]
    
    plt.plot(trial_data['time_normalized'], trial_data['robot_angle_z'], 
             marker='o', markersize=2, linewidth=1.5, 
             color=colors[i % len(colors)], 
             label=f'Trial {trial} (SSE: {sse:.2f}°)', alpha=0.8)

plt.title('All 5 Trials -5 Degree 90o Rotation')
plt.xlabel('Time (seconds)')
plt.ylabel('Robot Angle Z (degrees)')
plt.grid(True, alpha=0.3)
plt.axhline(y=5, color='r', linestyle='--', alpha=0.7, linewidth=2, label='Target: 5°')
plt.legend()
plt.tight_layout(pad= 10.0)
plt.show()

# Print some statistics for each trial including steady state error
print("Trial Statistics:")
print("=" * 50)
for trial in trials:
    trial_data = df[df['trial'] == trial]
    last_5_points = trial_data['robot_angle_z'].tail(5)
    avg_last_5 = last_5_points.mean()
    sse = steady_state_errors[trial]
    
    print(f"Trial {trial}:")
    print(f"  Data points: {len(trial_data)}")
    print(f"  Mean Z-angle: {trial_data['robot_angle_z'].mean():.2f}°")
    print(f"  Std Dev: {trial_data['robot_angle_z'].std():.2f}°")
    print(f"  Min: {trial_data['robot_angle_z'].min():.2f}°")
    print(f"  Max: {trial_data['robot_angle_z'].max():.2f}°")
    print(f"  Range: {trial_data['robot_angle_z'].max() - trial_data['robot_angle_z'].min():.2f}°")
    print(f"  Last 5 points average: {avg_last_5:.2f}°")
    print(f"  Steady State Error: {sse:.2f}°")
    print()