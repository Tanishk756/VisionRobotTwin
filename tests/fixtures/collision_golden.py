"""Deterministic Pre-A3 Collision and Planning Golden Numerical Baselines.

Captured from Phase A2 verified runtime.
Source commit: 26a7c187cf9d44c75c89b02ed3e7ef7e64ec17a7
Environment: Python 3.12.10, PyBullet 3.2.7 (Build Sep 13 2026 13:36:38)
"""

SOURCE_RUNTIME_SHA = "26a7c187cf9d44c75c89b02ed3e7ef7e64ec17a7"
PYBULLET_PACKAGE_VERSION = "3.2.7"
PYBULLET_BUILD_TIME = "Sep 13 2026 13:36:38"
PYTHON_VERSION = "3.12.10"

# Configurations for Collision Testing
PANDA_HOME_Q = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
PANDA_OBSTACLE_COLLISION_Q = [0.0, 0.04, 0.0, -2.38, 0.0, 2.41, 0.785]
PANDA_SELF_COLLISION_Q = [0.0, 1.5, 0.0, -3.0, 0.0, 3.5, 0.0]

KUKA_HOME_Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
KUKA_SELF_COLLISION_Q = [0.0, 2.0, 0.0, -2.0, 0.0, 2.0, 0.0]

# Planning test configurations
PLANNING_PANDA_START_Q = [0.2, -0.6, 0.1, -2.0, 0.1, 1.5, 0.7]
PLANNING_PANDA_GOAL_Q = [-0.2, -0.5, -0.1, -1.8, -0.1, 1.4, 0.6]
PLANNING_RRT_SEED = 42
PLANNING_SHORTCUT_SEED = 42
PLANNING_SHORTCUT_ATTEMPTS = 30
