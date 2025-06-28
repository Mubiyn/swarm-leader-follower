# 🤖 Clean Leader-Follower System

**A fresh, working implementation of multi-robot leader-follower navigation.**

## 🎯 What This Is

This is a **completely clean restart** of a leader-follower robot system. Unlike the complex existing codebase, this implementation is:

- ✅ **Simple and understandable**
- ✅ **Actually works out of the box**
- ✅ **Well documented**
- ✅ **Easy to extend**

## 🚀 Quick Start

### Option 1: Simple Demo (No ROS2 required)
```bash
# Just need Python + matplotlib
python simple_demo.py
```

### Option 2: Full ROS2 Version
```bash
# Activate your ROS2 environment
source activate_swarm_ros2.sh

# Run the full system
python clean_start.py
```

## 🎮 What You'll See

- 🔴 **Red circle**: Leader robot (moves in a circle)
- 🔵 **Blue circle**: Follower robot (tries to maintain 2m distance)
- **Gray dashed circle**: Target following distance
- **Colored trails**: Path history
- **Info boxes**: Real-time distance and velocity data

## 🏗️ System Architecture

### Simple Version (`simple_demo.py`)
```
Leader Position → Distance/Angle Calculation → Control Algorithm → Follower Movement
```

### ROS2 Version (`clean_start.py`)
```
Leader Node → /leader_position → Follower Node → /follower_cmd_vel → Visualization
```

## 🧠 How It Works

### 1. Leader Movement
The leader follows a simple circular path:
```python
x = 3.0 * cos(0.5 * t)
y = 3.0 * sin(0.5 * t)
```

### 2. Follower Control
The follower uses a simple proportional controller:
- **Distance error**: `distance - desired_distance`
- **Angle error**: `angle_to_leader - current_orientation`
- **Linear velocity**: Proportional to distance error
- **Angular velocity**: Proportional to angle error

### 3. Control Law
```python
if distance > desired_distance:
    linear_vel = k_linear * (distance - desired_distance)
else:
    linear_vel = 0.0

angular_vel = k_angular * angle_error
```

## 📊 Key Features

### ✅ Working Now
- [x] Leader-follower behavior
- [x] Distance maintenance (2m)
- [x] Real-time visualization
- [x] ROS2 integration
- [x] Clean, readable code

### 🔄 Easy Extensions
- [ ] Multiple followers
- [ ] Different formation patterns
- [ ] Obstacle avoidance
- [ ] Computer vision (robot detection)
- [ ] Advanced controllers (MPC, RL)

## 🛠️ Code Structure

### Core Classes

#### `LeaderRobot` (ROS2 version)
- Publishes position to `/leader_position`
- Moves in predefined patterns
- Can be controlled manually or programmatically

#### `FollowerRobot` (ROS2 version)
- Subscribes to `/leader_position`
- Publishes commands to `/follower_cmd_vel`
- Implements proportional control

#### `SimpleLeaderFollower` (No ROS2)
- Self-contained simulation
- Same control algorithm
- No external dependencies

### Key Parameters
```python
desired_distance = 2.0    # Target following distance (meters)
k_linear = 1.0           # Linear velocity gain
k_angular = 2.0          # Angular velocity gain
```

## 🎯 Next Steps

### Phase 1: Multiple Followers
```python
# Add more followers
followers = [FollowerRobot(f'robot_{i}') for i in range(3)]
```

### Phase 2: Formation Patterns
```python
# Different formations
formations = {
    'line': [(0, -2), (0, -4), (0, -6)],
    'triangle': [(0, -2), (-1.5, -3.5), (1.5, -3.5)],
    'circle': [polar_to_cart(2, angle) for angle in angles]
}
```

### Phase 3: Computer Vision
```python
# Replace leader position with detected position
def detect_leader(image):
    # YOLO detection or color detection
    return leader_position
```

### Phase 4: Advanced Control
```python
# MPC or RL controller
class MPCController:
    def compute_control(self, state, target):
        # Model predictive control
        return cmd_vel
```

## 🐛 Troubleshooting

### "No display" error
```bash
# For headless systems
export DISPLAY=:0
# Or use SSH with X forwarding
ssh -X user@robot
```

### ROS2 not found
```bash
# Check ROS2 installation
source activate_swarm_ros2.sh
ros2 doctor
```

### Matplotlib not working
```bash
# Install matplotlib
pip install matplotlib
# Or use conda
conda install matplotlib
```

## 🎓 Learning Resources

### Understanding the Control Algorithm
1. **Proportional Control**: Simple but effective
2. **Distance Regulation**: Maintains desired separation
3. **Angle Regulation**: Points towards leader
4. **Velocity Limiting**: Prevents unrealistic speeds

### ROS2 Concepts Used
1. **Nodes**: Independent processes
2. **Topics**: Communication channels  
3. **Publishers/Subscribers**: Message passing
4. **Timers**: Periodic callbacks

## 🤝 Contributing

This is designed to be **educational and extensible**. To add features:

1. **Start simple**: Test new features in `simple_demo.py` first
2. **Add ROS2 integration**: Move to `clean_start.py` 
3. **Keep it clean**: Document and test everything
4. **Make it modular**: Each feature should be independent

## 📝 License

Open source - feel free to use and modify!

---

**This is a clean, working foundation for multi-robot systems. Build on it! 🚀** 