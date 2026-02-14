# 🏁 F1TENTH NÜRBURGRING HACKATHON GUIDE
## Beginner-Friendly 4-Day Plan

---

## 📋 DAY 1: SETUP & BASELINE (Learn the System)

### Morning Session (2-3 hours)

#### Step 1: Setup Your Workspace
```bash
# Navigate to your F1TENTH workspace
cd ~/f1tenth_ws/src/

# Copy the improved code
cp /path/to/gap_finder_improved_v1.py ./your_package/scripts/
cp /path/to/config_nurburgring.yaml ./your_package/config/

# Make executable
chmod +x ./your_package/scripts/gap_finder_improved_v1.py

# Build
cd ~/f1tenth_ws
colcon build
source install/setup.bash
```

#### Step 2: Launch Configuration
Create a launch file `gap_finder_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('your_package'),
        'config',
        'config_nurburgring.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='your_package',
            executable='gap_finder_improved_v1.py',
            name='gap_follower',
            parameters=[config],
            output='screen'
        )
    ])
```

#### Step 3: First Test Run
```bash
# Terminal 1: Launch simulator with Nürburgring
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: Launch your node
ros2 launch your_package gap_finder_launch.py

# Terminal 3: Watch the visualizations in RViz
rviz2
```

#### Step 4: Record Baseline Performance
Create a testing log: `testing_log.md`

```markdown
# Nürburgring Testing Log

## Baseline Test (Original Parameters)
- Date: [DATE]
- Max Speed: 6.0 m/s
- Steering Gain: 0.5
- Result: ☐ Completed lap ☐ Crashed at: _______
- Lap Time: _______ seconds
- Issues Observed:
  - [ ] Too slow on straights
  - [ ] Crashes in corners
  - [ ] Oscillates/wobbles
  - [ ] Gets stuck
- Notes: _________________
```

### Afternoon Session (2-3 hours)

#### Step 5: Understand What You're Seeing

Watch the car in simulation and identify:
1. **In RViz, add these visualizations:**
   - `/scan` - Raw LIDAR data (red dots)
   - `/safety_bubble` - Obstacle avoidance zones (red spheres)
   - `/goal_point` - Where the car is aiming (green cylinder)
   - `/disparity_points` - Wall corners detected (pink spheres)

2. **In the terminal, watch the debug output:**
   ```
   Steer:  0.234 | Speed:  5.67 | Gap Width:  142 | Max Dist:  5.82m
   ```
   - **Steer**: Current steering angle (+ = left, - = right)
   - **Speed**: Current velocity
   - **Gap Width**: How wide the safe gap is
   - **Max Dist**: Distance to target point

#### Step 6: Identify Problem Areas
Watch the car complete (or fail) a lap and note:
- **Where does it slow down unnecessarily?** → Increase max_speed
- **Where does it crash?** → Increase safety bubbles or decrease speed
- **Does it wiggle/oscillate?** → Increase steering smoothing
- **Does it cut corners?** → Check disparity detection

---

## 📋 DAY 2: PARAMETER TUNING (Make It Faster)

### Goal: Find optimal parameters without writing code

#### Morning: Speed Optimization

**Test 1: Max Speed Sweep**
```yaml
# Edit config_nurburgring.yaml
max_speed: 5.0  # Test, record lap time
max_speed: 6.0  # Test, record lap time  
max_speed: 7.0  # Test, record lap time
max_speed: 8.0  # Test, record lap time (might crash!)
```

After each change:
1. Save the YAML file
2. Restart your node (Ctrl+C, then relaunch)
3. Time the lap
4. Record in your log

**Find the sweet spot**: Highest speed that completes laps reliably

**Test 2: Steering Aggressiveness**
```yaml
# Try different combinations:

# Conservative
steering_gain: 0.4
speed_gain: 1.0

# Balanced
steering_gain: 0.5
speed_gain: 0.7

# Aggressive
steering_gain: 0.6
speed_gain: 0.5
```

#### Afternoon: Safety vs Speed Tradeoff

**Test 3: Bubble Sizes**
```yaml
# Tight (risky but fast)
obstacle_bubble_radius: 0.10
disparity_bubble_radius: 0.10

# Medium
obstacle_bubble_radius: 0.12
disparity_bubble_radius: 0.12

# Safe (slow but reliable)
obstacle_bubble_radius: 0.15
disparity_bubble_radius: 0.15
```

**Test 4: Vision Range**
```yaml
# Short sight (reactive, for tight sections)
lookahead_distance: 4.0

# Medium
lookahead_distance: 6.0

# Long sight (smooth, for fast sections)
lookahead_distance: 8.0
```

### End of Day 2 Checkpoint
You should have:
- ✅ A working parameter set that completes laps
- ✅ A testing log with at least 10 different configurations
- ✅ Understanding of which parameters affect what

---

## 📋 DAY 3: ADVANCED FEATURES (Add Intelligence)

### Goal: Implement smarter behaviors

Now that you understand the basics, let's add more sophisticated features.

#### Morning: Corner Detection

Add this method to your improved code:

```python
def detect_corner_ahead(self, proc_ranges, data):
    """Detect if we're approaching a corner"""
    # Split the field of view into left and right halves
    mid_idx = len(proc_ranges) // 2
    
    left_ranges = proc_ranges[:mid_idx]
    right_ranges = proc_ranges[mid_idx:]
    
    # Calculate average distance on each side (ignore zeros)
    left_avg = np.mean(left_ranges[left_ranges > 0]) if np.any(left_ranges > 0) else 0
    right_avg = np.mean(right_ranges[right_ranges > 0]) if np.any(right_ranges > 0) else 0
    
    # If one side is much shorter, we're approaching a corner
    if left_avg > 0 and right_avg > 0:
        corner_severity = abs(left_avg - right_avg) / max(left_avg, right_avg)
        return corner_severity
    return 0.0
```

Then use it in `lidar_callback`:

```python
# After calculating raw_speed
corner_severity = self.detect_corner_ahead(proc_ranges, data)

# Brake if corner detected
if corner_severity > 0.3:  # Tight corner ahead
    msg.drive.speed *= 0.75  # Reduce speed by 25%
    if self.debug_mode:
        self.get_logger().warn(f'Corner detected! Severity: {corner_severity:.2f}')
```

**Test this**: Does it help with hairpins?

#### Afternoon: Gap Quality Scoring

Current algorithm just picks the furthest point. Let's make it smarter.

Add this method:

```python
def find_best_gap_advanced(self, proc_ranges, data):
    """Score gaps by both distance AND width"""
    gaps = []
    in_gap = False
    gap_start = 0
    
    for i, r in enumerate(proc_ranges):
        if r > 0 and not in_gap:
            gap_start = i
            in_gap = True
        elif r == 0 and in_gap:
            gap_end = i
            gap_width = gap_end - gap_start
            gap_depths = proc_ranges[gap_start:gap_end]
            gap_max_dist = np.max(gap_depths) if len(gap_depths) > 0 else 0
            gap_avg_dist = np.mean(gap_depths) if len(gap_depths) > 0 else 0
            
            # Score combines width and depth
            gap_score = gap_width * gap_avg_dist * gap_max_dist
            gaps.append((gap_score, gap_start, gap_end, gap_max_dist))
            in_gap = False
    
    if gaps:
        # Pick the highest scoring gap
        best_gap = max(gaps, key=lambda x: x[0])
        target_idx = (best_gap[1] + best_gap[2]) // 2
        
        # Get coordinates
        goal_distance = best_gap[3]
        goal_angle = data.angle_min + target_idx * data.angle_increment
        goal_x = goal_distance * np.cos(goal_angle)
        goal_y = goal_distance * np.sin(goal_angle)
        goal_coord = np.array([goal_x, goal_y])
        
        return target_idx, goal_coord
    
    # Fallback to original method
    return self.find_max_gap(data, proc_ranges)
```

**Create a parameter** to switch between methods:

```yaml
use_advanced_gap_finding: false  # Set true to test
```

**Compare**: Does the advanced method give better lap times?

---

## 📋 DAY 4: INTEGRATION & OPTIMIZATION (Race Day Prep)

### Morning: Race Configuration

#### Create specialized configs for different scenarios:

**1. Qualifying Config** (config_qualifying.yaml)
- Focus: Maximum single-lap speed
- Risk: Higher (willing to crash occasionally)

```yaml
max_speed: 8.0
obstacle_bubble_radius: 0.10
steering_gain: 0.6
smooth_steering: false  # Maximum responsiveness
```

**2. Race Config** (config_race.yaml)
- Focus: Consistency over 10+ laps
- Risk: Lower (must finish reliably)

```yaml
max_speed: 7.0
obstacle_bubble_radius: 0.13
steering_gain: 0.5
smooth_steering: true
```

#### Test Both Configurations:
- Run 5 consecutive laps with each
- Record: Completion rate, average lap time, fastest lap

### Afternoon: Final Optimization

#### Systematic Testing Protocol

1. **Create a test script**: `run_tests.sh`

```bash
#!/bin/bash

# Test different configurations automatically

CONFIGS=("config_conservative.yaml" "config_balanced.yaml" "config_aggressive.yaml")

for config in "${CONFIGS[@]}"
do
    echo "Testing $config..."
    # Launch with this config
    # Record results
    # Kill and restart
done
```

2. **Parameter Fine-Tuning Checklist:**

```
For your BEST configuration so far:

☐ Try max_speed ± 0.5 m/s
☐ Try steering_gain ± 0.05
☐ Try bubble_radius ± 0.01m
☐ Try lookahead ± 0.5m

Record if any improves lap time by >0.5s
```

3. **Edge Case Testing:**
   - Does it handle all corners?
   - Does it recover from near-crashes?
   - Can it complete 10 consecutive laps?

### Evening: Pre-Race Preparation

**Final Checklist:**

```markdown
## Pre-Race Checklist

Code:
- [ ] Set debug_mode: false (for performance)
- [ ] Remove unnecessary print statements
- [ ] Verify all parameters are in config file (not hardcoded)

Testing:
- [ ] 10 consecutive laps completed successfully
- [ ] Fastest lap time: ______ seconds
- [ ] Average lap time: ______ seconds
- [ ] Configuration file backed up

Race Strategy:
- [ ] Primary config chosen: ____________
- [ ] Backup config ready: ____________
- [ ] Know how to quick-swap configs if needed

Documentation:
- [ ] Testing log complete
- [ ] Key insights documented
- [ ] Parameter meanings understood
```

---

## 🎯 QUICK REFERENCE GUIDE

### Parameter Impact Cheat Sheet

| Parameter | Increase Effect | Decrease Effect |
|-----------|----------------|-----------------|
| max_speed | Faster but riskier | Slower but safer |
| steering_gain | Sharper turns | Gentler turns |
| speed_gain | Slower in turns | Faster in turns |
| lookahead_distance | Smoother, less reactive | More reactive, jerkier |
| obstacle_bubble_radius | Safer, more conservative | Riskier, more aggressive |
| steering_smoothing_alpha | More smoothing | Less smoothing |

### Common Problems & Solutions

**Problem: Car oscillates/wiggles**
→ Solution: Increase `steering_smoothing_alpha` to 0.4-0.5

**Problem: Crashes in corners**
→ Solution: Increase `obstacle_bubble_radius` or decrease `max_speed`

**Problem: Too slow on straights**
→ Solution: Increase `max_speed` and `lookahead_distance`

**Problem: Cuts corners**
→ Solution: Decrease `disparity_threshold` or increase `disparity_bubble_radius`

**Problem: Gets stuck**
→ Solution: Increase `field_of_vision` or decrease bubble sizes

---

## 🏆 WINNING STRATEGIES

### What Makes a Fast Lap?

1. **Smooth is fast**: Minimize steering oscillations
2. **Late braking**: Maintain speed as long as possible
3. **Early acceleration**: Get on the throttle coming out of corners
4. **Racing line**: The algorithm naturally finds it if parameters are right

### Competitive Edge Ideas

- **Test multiple parameter sets**: Don't commit to one too early
- **Watch other teams**: What behaviors do fast cars show?
- **Risk assessment**: Can you afford one crash for a 2-second improvement?
- **Track memory**: Advanced idea - map the track and use different parameters for different sections

---

## 📊 PERFORMANCE METRICS TO TRACK

```markdown
| Test # | Config | Max Speed | Steering Gain | Lap Time | Completed? | Notes |
|--------|--------|-----------|---------------|----------|------------|-------|
| 1      | Base   | 6.0       | 0.5           | 45.2s    | Yes        | Safe  |
| 2      | Aggr   | 7.0       | 0.6           | 41.8s    | Crash T3   | Too fast|
| ...    | ...    | ...       | ...           | ...      | ...        | ...   |
```

---

## 🚨 RACE DAY REMINDERS

1. **Don't make last-minute code changes**
2. **Have a backup configuration ready**
3. **Know how to quickly restart your node**
4. **Disable debug logging for performance**
5. **Trust your testing - don't panic and change things**

---

## 💡 LEARNING RESOURCES

### Understanding the Algorithm
- Draw the safety bubbles on paper
- Visualize why disparity detection works
- Understand the speed vs steering tradeoff

### ROS2 Basics
- How to check active topics: `ros2 topic list`
- How to echo messages: `ros2 topic echo /drive`
- How to see parameters: `ros2 param list`

### Debugging Tools
```bash
# See what's being published
ros2 topic echo /drive

# Check node status
ros2 node list
ros2 node info /gap_follower

# Monitor performance
ros2 topic hz /scan
```

---

Good luck! Remember: **Understanding beats random tuning every time.** 🏁
