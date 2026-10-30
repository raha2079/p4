#!/usr/bin/env python3
"""
Comprehensive visualization for checkpoint report
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def load_path(filename):
    """Load path from file"""
    try:
        data = np.loadtxt(filename)
        if len(data.shape) == 1:
            data = data.reshape(1, -1)
        return data
    except Exception as e:
        print(f"Warning: Could not load {filename}: {e}")
        return None

# ========================================
# FIGURE 1: Pendulum RRT vs KPIECE
# ========================================
def plot_pendulum_comparison():
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    torques = [3, 5, 10]
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # blue, orange, green

    for ax, planner in zip(axes, ['rrt', 'kpiece']):
        planner_name = planner.upper()

        for torque, color in zip(torques, colors):
            filename = f'pend_{planner}_t{torque}.txt'
            data = load_path(filename)

            if data is not None:
                theta, omega = data[:, 0], data[:, 1]

                # Plot path
                ax.plot(theta, omega, '-', color=color, linewidth=2.5,
                       label=f'τ = ±{torque}', alpha=0.8)

                # Mark start and goal
                ax.plot(theta[0], omega[0], 'o', color=color, markersize=10,
                       markeredgecolor='black', markeredgewidth=1.5)
                ax.plot(theta[-1], omega[-1], 's', color=color, markersize=10,
                       markeredgecolor='black', markeredgewidth=1.5)

        # Add reference lines
        ax.axhline(y=0, color='gray', linestyle='--', linewidth=1, alpha=0.4)
        ax.axvline(x=-np.pi/2, color='red', linestyle='--', linewidth=1.5, alpha=0.5, label='Start (θ=-π/2)')
        ax.axvline(x=np.pi/2, color='green', linestyle='--', linewidth=1.5, alpha=0.5, label='Goal (θ=+π/2)')
        ax.axhline(y=10, color='red', linestyle=':', linewidth=2, alpha=0.3)
        ax.axhline(y=-10, color='red', linestyle=':', linewidth=2, alpha=0.3, label='Velocity bounds (±10)')

        ax.set_xlabel('θ (angle) [rad]', fontsize=13, fontweight='bold')
        ax.set_ylabel('ω (angular velocity) [rad/s]', fontsize=13, fontweight='bold')
        ax.set_title(f'{planner_name} Planner', fontsize=15, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9, loc='best')
        ax.set_ylim(-12, 12)

    plt.suptitle('Pendulum Phase Space Trajectories: RRT vs KPIECE',
                 fontsize=17, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.savefig('pendulum_rrt_vs_kpiece.png', dpi=300, bbox_inches='tight')
    print("✓ Saved pendulum_rrt_vs_kpiece.png")
    plt.close()

# ========================================
# FIGURE 2: Car multiple configurations
# ========================================
def plot_car_configurations():
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()

    # Define obstacles (matching makeStreet() in C++)
    obstacles = [
        {'x': -6, 'y': 6, 'width': 12, 'height': 5, 'label': 'Top Wall'},      # Top wall
        {'x': -6, 'y': -11, 'width': 12, 'height': 5, 'label': 'Bottom Wall'}, # Bottom wall
        {'x': -6, 'y': -6, 'width': 1, 'height': 12, 'label': 'Left Wall'},    # Left wall
        {'x': 5, 'y': -6, 'width': 1, 'height': 12, 'label': 'Right Wall'},    # Right wall
        {'x': -4, 'y': -3, 'width': 3, 'height': 4, 'label': 'Left Obstacle'}, # Left middle obstacle
        {'x': 1, 'y': 2, 'width': 3, 'height': 3, 'label': 'Right Obstacle'},  # Right middle obstacle
    ]

    configs = [1, 3, 5, 7, 9]
    config_labels = [
        'Config 1: ω=0.5, a=1 (Very Hard)',
        'Config 3: ω=1, a=2 (Med-Hard)',
        'Config 5: ω=2, a=3 (Medium)',
        'Config 7: ω=3, a=4 (Easy)',
        'Config 9: ω=4, a=5 (Very Easy)'
    ]

    for idx, (config, label) in enumerate(zip(configs, config_labels)):
        ax = axes[idx]

        # Draw obstacles
        for obs in obstacles:
            rect = patches.Rectangle(
                (obs['x'], obs['y']), obs['width'], obs['height'],
                linewidth=1.5, edgecolor='black', facecolor='gray', alpha=0.4
            )
            ax.add_patch(rect)

        # Try RRT first, then KPIECE
        for planner, color, style in [('rrt', 'blue', '-'), ('kpiece', 'red', '--')]:
            filename = f'car_{planner}_c{config}.txt'
            data = load_path(filename)

            if data is not None:
                x, y = data[:, 0], data[:, 1]
                theta = data[:, 2]

                ax.plot(x, y, style, color=color, linewidth=2,
                       label=planner.upper(), alpha=0.8)

                # Mark start/goal
                if planner == 'rrt':
                    ax.plot(x[0], y[0], 'go', markersize=12, markeredgecolor='darkgreen',
                           markeredgewidth=2, label='Start', zorder=10)
                    ax.plot(x[-1], y[-1], 'rs', markersize=12, markeredgecolor='darkred',
                           markeredgewidth=2, label='Goal', zorder=10)

        ax.set_xlim(-6, 6)
        ax.set_ylim(-11, 11)
        ax.set_xlabel('X [m]', fontsize=11, fontweight='bold')
        ax.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
        ax.set_title(label, fontsize=11, fontweight='bold')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')

    # Hide the 6th subplot
    axes[5].axis('off')

    plt.suptitle('Car Navigation: Multiple Control Configurations',
                 fontsize=18, fontweight='bold')
    plt.tight_layout()
    plt.savefig('car_multiple_configs.png', dpi=300, bbox_inches='tight')
    print("✓ Saved car_multiple_configs.png")
    plt.close()

# ========================================
# FIGURE 3: Torque comparison detail
# ========================================
def plot_torque_analysis():
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    torques = [3, 5, 10]
    titles = [
        'Low Torque (τ=±3)\nRequires Momentum Building',
        'Medium Torque (τ=±5)\nBalanced Approach',
        'High Torque (τ=±10)\nDirect Power'
    ]

    for ax, torque, title in zip(axes, torques, titles):
        # Plot both RRT and KPIECE
        for planner, color, style in [('rrt', 'blue', '-'), ('kpiece', 'red', '--')]:
            filename = f'pend_{planner}_t{torque}.txt'
            data = load_path(filename)

            if data is not None:
                theta, omega = data[:, 0], data[:, 1]
                ax.plot(theta, omega, style, color=color, linewidth=2.5,
                       label=planner.upper(), alpha=0.8)

                # Mark endpoints
                ax.plot(theta[0], omega[0], 'o', color=color, markersize=10)
                ax.plot(theta[-1], omega[-1], 's', color=color, markersize=10)

        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.4)
        ax.axvline(x=-np.pi/2, color='red', linestyle='--', alpha=0.5)
        ax.axvline(x=np.pi/2, color='green', linestyle='--', alpha=0.5)
        ax.set_xlabel('θ [rad]', fontsize=12, fontweight='bold')
        ax.set_ylabel('ω [rad/s]', fontsize=12, fontweight='bold')
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=10)
        ax.set_ylim(-12, 12)

    plt.suptitle('Pendulum Behavior Analysis: Effect of Torque Limits',
                 fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig('pendulum_torque_analysis.png', dpi=300, bbox_inches='tight')
    print("✓ Saved pendulum_torque_analysis.png")
    plt.close()

if __name__ == '__main__':
    print("Generating comprehensive visualizations...")
    plot_pendulum_comparison()
    plot_car_configurations()
    plot_torque_analysis()
    print("Done!")
