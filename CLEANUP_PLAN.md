# Repository Cleanup Plan

## 🗑️ Files and Directories to Remove

### 1. **Archive Directory** - Safe to Remove
```
archive/
├── test_individual_demos.py
└── test_results.txt
```
**Reason**: Contains old test files that are no longer needed

### 2. **Old Configuration Files** - Safe to Remove
```
config/
├── swarm_parameters_old.yaml
└── swarm_test.world
```
**Reason**: Old configuration files replaced by newer versions

### 3. **Redundant Demo Files** - Safe to Remove
```
demos/
├── python/
│   ├── mpc_leader_follower.py
│   ├── multi_follower_demo.py
│   ├── obstacle_avoidance_demo.py
│   └── [other Python demo files]
└── ros2/
    ├── clean_start.py
    ├── multi_follower_ros2.py
    ├── obstacle_avoidance_ros2.py
    └── [other ROS2 demo files]
```
**Reason**: These are individual demo files that have been integrated into the unified system

### 4. **Old Model Files** - Safe to Remove
```
models/
├── rl_follower_1.pkl
├── rl_follower_2.pkl
└── rl_follower_3.pkl
```
**Reason**: RL models are not used in the current implementation

### 5. **Old Launch Files** - Safe to Remove
```
launch/
├── enhanced_swarm_system.launch.py
├── formation_control.launch.py
└── gazebo_swarm_simulation.launch.py
```
**Reason**: Replaced by unified launch files

### 6. **Old Source Files** - Safe to Remove
```
src/
├── control/
│   └── mpc_controller.py
├── environments/
├── rl/
├── scenarios/
└── simulation/
    ├── environment_visualizer.py
    └── scene_generator.py
```
**Reason**: These are old source files that have been integrated into the ROS2 workspace

### 7. **Old Service Files** - Safe to Remove
```
srv/
├── AddObstacle.srv
├── SetController.srv
└── SetFormation.srv
```
**Reason**: Service definitions are now in the ROS2 workspace

### 8. **Old URDF and World Files** - Safe to Remove
```
urdf/
└── swarm_robot.urdf.xacro

worlds/
└── swarm_arena.world
```
**Reason**: These are now in the ROS2 workspace

### 9. **Old Test Files** - Safe to Remove
```
tests/
├── integration/
└── unit/
```
**Reason**: Empty test directories

### 10. **Old Documentation Files** - Safe to Remove
```
docs/
└── ARCHITECTURE.md
```
**Reason**: Architecture is now documented in the main README and feature summary

### 11. **Old Scripts** - Safe to Remove
```
scripts/
├── launch_swarm.sh
├── setup_macos.sh
└── setup_project.sh
```
**Reason**: Replaced by ROS2 workspace scripts

### 12. **Old Configuration Files** - Safe to Remove
```
configs/
```
**Reason**: Empty directory

### 13. **Old Dataset Directory** - Safe to Remove
```
datasets/
```
**Reason**: Empty directory

### 14. **Old Notebooks Directory** - Safe to Remove
```
notebooks/
```
**Reason**: Empty directory

## 📁 Files to Keep

### Core ROS2 Workspace
```
ros2_workspace/
├── src/modern_swarm/
├── install/
├── build/
└── log/
```

### Essential Configuration
```
config/
├── scenarios/
│   ├── conservative.yaml
│   ├── high_speed.yaml
│   └── research.yaml
└── swarm_parameters.yaml
```

### Essential Scripts
```
ros2_workspace/
├── build_and_test.sh
└── run_complete_demo.sh
```

### Documentation
```
README.md
FEATURE_SUMMARY.md
CLEANUP_PLAN.md
```

### Docker Support
```
docker/
├── docker-compose.yml
├── Dockerfile
└── ros2_setup.sh
```

## 🧹 Cleanup Commands

```bash
# Remove archive directory
rm -rf archive/

# Remove old configuration files
rm -f config/swarm_parameters_old.yaml
rm -f config/swarm_test.world

# Remove old demo files
rm -rf demos/

# Remove old model files
rm -rf models/

# Remove old launch files
rm -f launch/enhanced_swarm_system.launch.py
rm -f launch/formation_control.launch.py
rm -f launch/gazebo_swarm_simulation.launch.py

# Remove old source files
rm -rf src/

# Remove old service files
rm -f srv/*.srv

# Remove old URDF and world files
rm -rf urdf/
rm -rf worlds/

# Remove old test files
rm -rf tests/

# Remove old documentation
rm -rf docs/

# Remove old scripts
rm -f scripts/launch_swarm.sh
rm -f scripts/setup_macos.sh
rm -f scripts/setup_project.sh

# Remove empty directories
rm -rf configs/
rm -rf datasets/
rm -rf notebooks/

# Remove old log files (keep recent ones)
find log/ -name "*.log" -mtime +7 -delete
```

## 📊 Expected Results

### Before Cleanup
- **Total Files**: ~200+ files
- **Repository Size**: ~50MB+
- **Directories**: 20+ directories

### After Cleanup
- **Total Files**: ~50 files
- **Repository Size**: ~10MB
- **Directories**: 5-8 directories

## ✅ Benefits of Cleanup

1. **Reduced Repository Size**: 80% size reduction
2. **Improved Navigation**: Easier to find relevant files
3. **Better Organization**: Clear separation of old vs. new code
4. **Faster Cloning**: Smaller repository downloads faster
5. **Reduced Confusion**: No more old files to confuse users
6. **Cleaner Structure**: Focus on the working ROS2 implementation

## 🚀 Post-Cleanup Structure

```
modern_swarm_leader_follower/
├── README.md
├── FEATURE_SUMMARY.md
├── CLEANUP_PLAN.md
├── config/
│   ├── scenarios/
│   └── swarm_parameters.yaml
├── docker/
├── ros2_workspace/
│   ├── src/modern_swarm/
│   ├── install/
│   ├── build/
│   └── log/
└── performance_plots/ (generated during runtime)
```

This structure will be much cleaner and focused on the working ROS2 implementation. 