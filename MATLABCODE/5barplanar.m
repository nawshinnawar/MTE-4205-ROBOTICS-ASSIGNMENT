import sys
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.backend_bases

# Manipulator link lengths
l0 = 6.0    # Distance from the origin to left/right motor centers
l1 = 8.0    # Length from motor to passive joint
l2 = 8.0    # Length from passive joint to end effector

# Define the Y_MIN for the workspace calculation
Y_MIN = -15  # Adjust as needed

def calc_angles(x, y):
    # Angle from left shoulder to end effector
    beta1 = math.atan2( y, (l0 + x) )

    # Angle from right shoulder to end effector
    beta2 = math.atan2( y, (l0 - x) )

    # Alpha angle pre-calculations
    alpha1_calc = (l1**2 + ( (l0 + x)**2 + y**2 ) - l2**2) / (2*l1*math.sqrt( (l0 + x)**2 + y**2 ))
    alpha2_calc = (l1**2 + ( (l0 - x)**2 + y**2 ) - l2**2) / (2*l1*math.sqrt( (l0 - x)**2 + y**2 ))

    # If calculations > 1 or < -1, will fail acos function
    if alpha1_calc > 1 or alpha1_calc < -1 or alpha2_calc > 1 or alpha2_calc < -1:
        return None, None  # Indicate unreachable
    else:
        # Angle of left shoulder - beta1 and right shoulder - beta2
        alpha1 = math.acos(alpha1_calc)
        alpha2 = math.acos(alpha2_calc)

        # Angles of left and right shoulders
        shoulder1 = beta1 + alpha1
        shoulder2 = math.pi - beta2 - alpha2

        return(shoulder1, shoulder2)

def plot_arms(shoulder1, shoulder2, efx, efy):
    # Fixed points (motors)
    a1 = (-l0, 0)
    a2 = (l0, 0)

    # Passive joints (x, y) location
    p1 = ( -l0 + l1*math.cos(shoulder1), l1*math.sin(shoulder1)  )
    p2 = ( l0 + l1*math.cos(shoulder2), l1*math.sin(shoulder2)  )

    # End effector
    e = (efx, efy)

    # Unpack coordinates
    a1_x, a1_y = a1
    a2_x, a2_y = a2
    p1_x, p1_y = p1
    p2_x, p2_y = p2
    e_x, e_y = e

    # Left arm
    plt.plot([a1_x, p1_x, e_x], [a1_y, p1_y, e_y], 'bo-')
    plt.text(a1_x - 0.5, a1_y + 0.3, "A1")
    plt.text(p1_x + 0.3, p1_y + 0.3, "P1 ({:.2f}, {:.2f})".format(p1_x, p1_y), fontsize=8)
    plt.text(-l0+0.3, 0+0.3, "{:.2f} degrees".format(math.degrees(shoulder1)), fontsize=8)

    # Right arm
    plt.plot([a2_x, p2_x, e_x], [a2_y, p2_y, e_y], 'bo-')
    plt.text(a2_x + 0.3, a2_y + 0.3, "A2")
    plt.text(p2_x + 0.3, p2_y + 0.3, "P2 ({:.2f}, {:.2f})".format(p2_x, p2_y), fontsize=8)
    plt.text(l0+0.3, 0+0.3, "{:.2f} degrees".format(math.degrees(shoulder2)), fontsize=8)

    # EF
    plt.plot(e_x, e_y, 'ro')
    plt.text(e_x + 0.3, e_y + 0.3, "E ({:.2f}, {:.2f})".format(e_x, e_y), fontsize=8)

def on_press(event):
    if event.key == 'q':
        sys.exit()

def compute_workspace_boundary():
    """
    Computes a grid over the approximate workspace region and determines
    which (x, y) points are reachable by the robot.

    The robot is reachable at a point (x,y) if the distance between (x,y)
    and each motor (located at (-l0, 0) and (l0, 0)) is in the range:
        [|l1 - l2|, l1+l2].
    Returns:
        X, Y: Meshgrid arrays.
        reach: A 2D array with 1 for reachable and 0 for unreachable.
    """
    x_vals = np.linspace(-20, 20, 400)
    y_vals = np.linspace(Y_MIN, 20, 400)
    X, Y = np.meshgrid(x_vals, y_vals)
    r1 = np.sqrt((X + l0)**2 + Y**2)
    r2 = np.sqrt((l0 - X)**2 + Y**2)
    # Reachable condition for each chain
    cond1 = np.logical_and(r1 >= abs(l1 - l2), r1 <= (l1 + l2))
    cond2 = np.logical_and(r2 >= abs(l1 - l2), r2 <= (l1 + l2))
    reach = np.logical_and(cond1, cond2).astype(float)
    return X, Y, reach

def plot_plot(efx, efy, workspace_x, workspace_y, reach):
    plt.title('5-Bar Parallel Robot Kinematics')
    plt.contourf(workspace_x, workspace_y, reach, levels=[0.5, 1], colors=['lightgray'], alpha=0.5, label='Approximate Workspace')
    plt.plot(-20, -20, 'wo') # Invisible point to set lower bounds
    plt.plot(20, 20, 'wo')  # Invisible point to set upper bounds

    s1, s2 = calc_angles(efx, efy)
    if s1 is not None and s2 is not None:
        plot_arms(s1, s2, efx, efy)
        print(f"Coordinates Reachable: ({efx:.2f}, {efy:.2f})")
    else:
        plt.plot(efx, efy, 'rx', markersize=10, label='Unreachable') # Mark unreachable points

    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.draw()
    plt.pause(0.1)
    plt.clf()

if __name__ == "__main__":
    fig = plt.figure()
    fig.canvas.mpl_connect('key_press_event', on_press)

    workspace_x, workspace_y, reach = compute_workspace_boundary()

    a1_x, a1_y = -l0, 0
    a2_x, a2_y = l0, 0

    while True:
        for i in range(-15, 16):
            plot_plot(i, 10, workspace_x, workspace_y, reach)
        for j in range(10, 16):
            plot_plot(15, j, workspace_x, workspace_y, reach)
        for k in range(15, -16, -1):
            plot_plot(k, 15, workspace_x, workspace_y, reach)
        for l in range(15, 9, -1):
            plot_plot(-15, l, workspace_x, workspace_y, reach)