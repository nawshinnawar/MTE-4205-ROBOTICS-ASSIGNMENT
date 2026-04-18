import sys
import math
import matplotlib.pyplot as plt
import numpy as np

# Manipulator link lengths
l0 = 6.0    # Distance from origin to motor centers (Total distance between motors = 12.0)
l1 = 8.0    # Length of proximal arm (driven by motor)
l2 = 8.0    # Length of distal arm (connected to end effector)
Y_MIN = -5  # Adjusted for better workspace visualization

def calc_angles(x, y):
    # Distance from left motor (-l0, 0) to end effector
    d1 = math.sqrt((x + l0)**2 + y**2)
    # Distance from right motor (l0, 0) to end effector
    d2 = math.sqrt((x - l0)**2 + y**2)

    # Reachability check: Point must be within [|l1-l2|, l1+l2] for both arms
    if d1 > (l1 + l2) or d1 < abs(l1 - l2) or d2 > (l1 + l2) or d2 < abs(l1 - l2):
        return None, None

    # Left Shoulder Calculation
    beta1 = math.atan2(y, x + l0)
    # Law of Cosines: alpha is the interior angle at the motor
    alpha1 = math.acos((l1**2 + d1**2 - l2**2) / (2 * l1 * d1))
    shoulder1 = beta1 + alpha1  # "Elbow-up" configuration

    # Right Shoulder Calculation
    beta2 = math.atan2(y, x - l0)
    alpha2 = math.acos((l1**2 + d2**2 - l2**2) / (2 * l1 * d2))
    shoulder2 = beta2 - alpha2  # Mirror configuration for the right side

    return shoulder1, shoulder2

def plot_arms(shoulder1, shoulder2, efx, efy):
    # Fixed motor positions
    a1_x, a1_y = -l0, 0
    a2_x, a2_y = l0, 0
    
    # Passive joint (elbow) locations
    p1_x, p1_y = -l0 + l1 * math.cos(shoulder1), l1 * math.sin(shoulder1)
    p2_x, p2_y = l0 + l1 * math.cos(shoulder2), l1 * math.sin(shoulder2)
    
    # Plot Left Arm (Motor -> Elbow -> EF)
    plt.plot([a1_x, p1_x, efx], [a1_y, p1_y, efy], 'r-o', linewidth=2, label="Left Arm")
    # Plot Right Arm (Motor -> Elbow -> EF)
    plt.plot([a2_x, p2_x, efx], [a2_y, p2_y, efy], 'b-o', linewidth=2, label="Right Arm")
    
    # Plot End Effector
    plt.plot(efx, efy, 'go', markersize=8)

def compute_workspace_boundary():
    x_vals = np.linspace(-20, 20, 200)
    y_vals = np.linspace(Y_MIN, 20, 200)
    X, Y = np.meshgrid(x_vals, y_vals)
    
    r1 = np.sqrt((X + l0)**2 + Y**2)
    r2 = np.sqrt((X - l0)**2 + Y**2)
    
    cond1 = (r1 >= abs(l1 - l2)) & (r1 <= (l1 + l2))
    cond2 = (r2 >= abs(l1 - l2)) & (r2 <= (l1 + l2))
    reach = (cond1 & cond2).astype(float)
    return X, Y, reach

def update_plot(efx, efy, X, Y, reach):
    plt.cla()  # Clear axes, not the whole figure
    plt.contourf(X, Y, reach, levels=[0.5, 1], colors=['#f0f0f0'])
    plt.title(f'5-Bar Robot Kinematics | EF: ({efx}, {efy})')
    
    s1, s2 = calc_angles(efx, efy)
    if s1 is not None:
        plot_arms(s1, s2, efx, efy)
    else:
        plt.plot(efx, efy, 'rx', markersize=10, label="Out of Reach")
    
    plt.xlim(-18, 18)
    plt.ylim(Y_MIN, 18)
    plt.gca().set_aspect('equal')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.pause(0.01)

if __name__ == "__main__":
    plt.figure(figsize=(8, 6))
    X, Y, reach = compute_workspace_boundary()
    
    try:
        while True:
            # Create a simple rectangular path for the end effector
            # Bottom edge
            for x in range(-10, 11): update_plot(x, 5, X, Y, reach)
            # Right edge
            for y in range(5, 13): update_plot(10, y, X, Y, reach)
            # Top edge
            for x in range(10, -11, -1): update_plot(x, 12, X, Y, reach)
            # Left edge
            for y in range(12, 4, -1): update_plot(-10, y, X, Y, reach)
    except KeyboardInterrupt:
        print("Simulation stopped.")