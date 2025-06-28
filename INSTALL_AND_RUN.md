# 🚀 Installation and Running Instructions

## 📋 **Step-by-Step Setup**

### **Step 1: Clean Up Project Structure**

First, organize the messy directory:

```bash
cd modern_swarm_leader_follower
python cleanup_project.py
```

This will organize files into:
- `demos/python/` - Pure Python demos
- `demos/ros2/` - ROS2 integrated demos  
- `archive/` - Old experimental files

### **Step 2: Set Up ROS2 Environment**

Run the automated setup script:

```bash
chmod +x setup_ros2_workspace.sh
./setup_ros2_workspace.sh
```

This script will:
- ✅ Create/activate conda environment `swarm_ros2`
- ✅ Install ROS2 Humble via conda-forge
- ✅ Install all Python dependencies
- ✅ Create ROS2 workspace at `~/ros2_ws`
- ✅ Build the modern_swarm package
- ✅ Create activation script

### **Step 3: Activate Environment**

Navigate to the workspace and activate:

```bash
cd ~/ros2_ws
source activate_modern_swarm.sh
```

You should see:
```
🤖 Activating Modern Swarm ROS2 Environment...
✅ Modern Swarm Environment Ready!
```

---

## 🎮 **Running the System**

### **Option A: Pure Python Demos (No ROS2 required)**

```bash
cd ~/ros2_ws/src/modern_swarm
python demos/python/simple_demo.py
python demos/python/multi_follower_demo.py  # Press SPACEBAR to switch formations!
python demos/python/vision_leader_follower.py  # Press V to toggle vision!
```

### **Option B: ROS2 Integration**

#### **Basic ROS2 Demo:**
```bash
# Terminal 1: Launch the swarm controller
ros2 launch modern_swarm simple_swarm_demo.launch.py

# Terminal 2: Monitor the system
ros2 topic list
ros2 topic echo /swarm_status
```

#### **With RViz Visualization:**
```bash
ros2 launch modern_swarm simple_swarm_demo.launch.py use_rviz:=true
```

#### **With Gazebo Simulation:**
```bash
ros2 launch modern_swarm simple_swarm_demo.launch.py use_gazebo:=true use_rviz:=true
```

### **Option C: Interactive Control**

Switch formations during runtime:
```bash
# Terminal 1: Launch system
ros2 launch modern_swarm simple_swarm_demo.launch.py use_rviz:=true

# Terminal 2: Control formations
ros2 service call /switch_formation std_srvs/srv/SetBool "{data: true}"
```

---

## 🔧 **Troubleshooting**

### **Common Issues:**

#### **1. "Package not found" error:**
```bash
cd ~/ros2_ws
colcon build --packages-select modern_swarm
source install/setup.bash
```

#### **2. RViz won't start:**
```bash
# Check if RViz is installed in your conda environment
conda list | grep rviz
# If not found:
conda install ros-humble-rviz2 -y
```

#### **3. Gazebo issues:**
```bash
# Check Gazebo installation
conda list | grep gazebo
# If not found:
conda install ros-humble-gazebo-ros-pkgs -y
```

#### **4. Import errors:**
```bash
# Check Python packages
pip install -r requirements.txt
# Or install missing packages individually:
pip install numpy matplotlib opencv-python
```

### **Environment Check:**

Run this to verify your setup:
```bash
# Check ROS2
echo $ROS_DISTRO  # Should show: humble

# Check packages
ros2 pkg list | grep modern_swarm  # Should show: modern_swarm

# Check nodes
ros2 run modern_swarm swarm_controller --help
```

---

## 📊 **Available Commands**

### **ROS2 Commands:**
```bash
# Launch basic system
ros2 launch modern_swarm simple_swarm_demo.launch.py

# Monitor topics
ros2 topic list
ros2 topic echo /leader/pose
ros2 topic echo /swarm_status

# Control formations
ros2 service call /switch_formation std_srvs/srv/SetBool "{data: true}"

# Check node status
ros2 node list
ros2 node info /swarm_controller
```

### **Python Demos:**
```bash
cd ~/ros2_ws/src/modern_swarm
python demos/python/simple_demo.py           # Basic 1v1 demo
python demos/python/multi_follower_demo.py   # Formation switching (SPACEBAR)
python demos/python/vision_leader_follower.py # Vision system (V key)
python demos/python/obstacle_avoidance_demo.py # Obstacle avoidance
python demos/python/mpc_leader_follower.py   # Advanced MPC control (M key)
python demos/python/rl_leader_follower.py    # Reinforcement learning (R key)
```

---

## 🎯 **Verification Steps**

### **1. Basic Function Test:**
```bash
# This should work without any errors:
cd ~/ros2_ws
source activate_modern_swarm.sh
python src/modern_swarm/demos/python/simple_demo.py
```

### **2. ROS2 Integration Test:**
```bash
# This should launch RViz with robot visualization:
ros2 launch modern_swarm simple_swarm_demo.launch.py use_rviz:=true
```

### **3. Formation Control Test:**
```bash
# Terminal 1:
ros2 launch modern_swarm simple_swarm_demo.launch.py use_rviz:=true

# Terminal 2 (wait 5 seconds, then run):
ros2 service call /switch_formation std_srvs/srv/SetBool "{data: true}"
```

You should see the formation change in RViz!

---

## 🏆 **Success Criteria**

✅ **Level 1: Basic Python Demos Work**
- Can run `simple_demo.py` without errors
- Matplotlib window shows leader-follower simulation

✅ **Level 2: ROS2 Integration Works**
- Can launch ROS2 nodes without errors
- `ros2 topic list` shows robot topics

✅ **Level 3: RViz Visualization Works**  
- RViz launches and shows robot poses
- Robot movements are visible in real-time

✅ **Level 4: Formation Control Works**
- Can switch formations via ROS2 services
- Formation changes are visible in RViz

✅ **Level 5: Full Gazebo Integration**
- Gazebo launches with robot models
- Physics simulation works correctly

---

## 📞 **Getting Help**

If you encounter issues:

1. **Check the logs:**
   ```bash
   ros2 launch modern_swarm simple_swarm_demo.launch.py 2>&1 | tee log.txt
   ```

2. **Verify environment:**
   ```bash
   conda list | grep ros
   echo $ROS_DISTRO
   echo $PYTHONPATH
   ```

3. **Test individual components:**
   ```bash
   # Test Python environment
   python -c "import rclpy, numpy, matplotlib; print('✅ All imports work!')"
   
   # Test ROS2 installation
   ros2 --help
   ```

4. **Clean rebuild:**
   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build --packages-select modern_swarm
   ```

---

## 🎉 **Next Steps After Setup**

Once everything is working:

1. **Explore the demos** in `demos/python/`
2. **Experiment with ROS2 integration** using the launch files
3. **Modify formation patterns** in the code
4. **Add new robot behaviors** using the modular architecture
5. **Integrate with real robots** by modifying the ROS2 topics

**You now have a fully functional modern swarm robotics system!** 🚀 