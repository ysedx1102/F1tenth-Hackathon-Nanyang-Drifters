# CS STUDENT - UPDATED TASK LIST (4-PERSON TEAM)
## Role: Algorithm Development & Control Optimization

**Updated Time**: 28 hours (+8h from EE departure)

---

## ⚠️ WHAT CHANGED

You're now responsible for:
- ✅ **Data logging** (was EE's task)
- ✅ **Steering smoothing tuning** (was EE's task)
- ✅ **Speed control optimization** (was EE's task)

**Why you?** You have the coding skills and already understand the algorithm deeply.

**Don't worry**: These are mostly parameter tuning, not complex new code.

---

## 📅 UPDATED TASK BREAKDOWN

### Week 1 Tasks (10 hours total)

#### Day 1-2: Code Analysis (3 hours) - UNCHANGED
- [ ] Read entire gap_finder_base.py
- [ ] Annotate code with comments
- [ ] Identify improvement opportunities
- [ ] Research numpy optimization

**Deliverable**: Annotated code + improvement ideas

---

#### Day 6-7: Core Improvements + Data Logging (5 hours) - ADDED 1h
- [ ] Implement steering smoothing (1h)
- [ ] Implement adaptive speed (1h)  
- [ ] Change to quadratic steering penalty (30min)
- [ ] **NEW: Add data logging** (1.5h)
- [ ] Test each feature (1h)

**NEW: Data Logging Implementation**:
```python
import csv
from datetime import datetime

class PerformanceLogger:
    def __init__(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'race_log_{timestamp}.csv'
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        # Write header
        self.writer.writerow([
            'timestamp',
            'steering_angle',
            'speed',
            'gap_width',
            'max_distance',
            'min_distance'
        ])
    
    def log_data(self, timestamp, steering, speed, gap_width, 
                 max_dist, min_dist):
        self.writer.writerow([
            f'{timestamp:.3f}',
            f'{steering:.4f}',
            f'{speed:.4f}',
            gap_width,
            f'{max_dist:.4f}',
            f'{min_dist:.4f}'
        ])
        self.file.flush()  # Ensure data is written immediately
    
    def close(self):
        self.file.close()

# Add to GapFollower class __init__:
self.logger = PerformanceLogger()

# Add to lidar_callback:
timestamp = self.get_clock().now().nanoseconds / 1e9
min_dist = np.min(proc_ranges[proc_ranges > 0]) if np.any(proc_ranges > 0) else 0

self.logger.log_data(
    timestamp,
    msg.drive.steering_angle,
    msg.drive.speed,
    gap_width,
    proc_ranges[max_dist_index] if max_dist_index else 0,
    min_dist
)

# Add to shutdown:
def __del__(self):
    if hasattr(self, 'logger'):
        self.logger.close()
```

**Deliverable**: Updated gap_finder with logging

---

### Week 2 Tasks (18 hours total)

#### Day 8-9: Corner Detection + Steering Tuning (5 hours) - ADDED 2h

**Original Task: Corner Detection (3h)**:
```python
def detect_corner_ahead(self, proc_ranges):
    """Detect approaching corners by asymmetry"""
    mid_idx = len(proc_ranges) // 2
    
    left_ranges = proc_ranges[:mid_idx]
    right_ranges = proc_ranges[mid_idx:]
    
    left_avg = np.mean(left_ranges[left_ranges > 0]) if np.any(left_ranges > 0) else 0
    right_avg = np.mean(right_ranges[right_ranges > 0]) if np.any(right_ranges > 0) else 0
    
    if left_avg > 0 and right_avg > 0:
        return abs(left_avg - right_avg) / max(left_avg, right_avg)
    return 0.0

# In lidar_callback:
corner_severity = self.detect_corner_ahead(proc_ranges)
if corner_severity > 0.3:
    msg.drive.speed *= 0.75  # Brake for corners
```

**NEW TASK: Steering Smoothing Tuning (2h)**:

Work with Mech Eng #1 to find optimal smoothing:

```python
# Test different alpha values
smoothing_alphas_to_test = [0.1, 0.2, 0.3, 0.4, 0.5]

# For each alpha:
# 1. Update parameter in config file
# 2. Mech #1 runs 2 test laps
# 3. Observe oscillation behavior
# 4. Record results

# Create comparison table:
"""
| Alpha | Lap Time | Oscillations | Path Smoothness | Notes |
|-------|----------|--------------|-----------------|-------|
| 0.1   |          |              |                 |       |
| 0.2   |          |              |                 |       |
| 0.3   |          |              |                 |       |
| 0.4   |          |              |                 |       |
| 0.5   |          |              |                 |       |
"""

# Choose best value based on:
# - Minimum oscillations
# - Acceptable lap time (shouldn't be much slower)
# - Smooth steering transitions
```

**Deliverable**: Corner detection code + optimal smoothing parameter

---

#### Day 10-11: Advanced Gap Finding + Speed Control (7 hours) - ADDED 3h

**Original: Advanced Gap Finding (4h)**:
```python
def find_best_gap_advanced(self, proc_ranges, data):
    """Score gaps by combining width and depth"""
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
            
            if len(gap_depths) > 0:
                gap_max_dist = np.max(gap_depths)
                gap_avg_dist = np.mean(gap_depths)
                gap_score = gap_width * gap_avg_dist * gap_max_dist
                gaps.append((gap_score, gap_start, gap_end, gap_max_dist))
            in_gap = False
    
    if gaps:
        best_gap = max(gaps, key=lambda x: x[0])
        target_idx = (best_gap[1] + best_gap[2]) // 2
        
        goal_distance = best_gap[3]
        goal_angle = data.angle_min + target_idx * data.angle_increment
        goal_x = goal_distance * np.cos(goal_angle)
        goal_y = goal_distance * np.sin(goal_angle)
        goal_coord = np.array([goal_x, goal_y])
        
        return target_idx, goal_coord
    
    return self.find_max_gap(data, proc_ranges)
```

**NEW: Speed Control Optimization (3h)**:

Test different speed control formulas:

```python
# Current (Quadratic):
def speed_control_quadratic(self, max_speed, steering_angle, speed_gain):
    raw_speed = max_speed - (abs(steering_angle) ** 2) * speed_gain
    return raw_speed

# Option 2 (Piecewise - straight vs turning):
def speed_control_piecewise(self, max_speed, steering_angle, speed_gain):
    STRAIGHT_THRESHOLD = 0.1  # radians
    
    if abs(steering_angle) < STRAIGHT_THRESHOLD:
        # Going straight - full speed
        return max_speed
    else:
        # Turning - reduce speed
        return max_speed - (abs(steering_angle) ** 2) * speed_gain

# Option 3 (Exponential decay):
def speed_control_exponential(self, max_speed, steering_angle, speed_gain):
    return max_speed * np.exp(-speed_gain * abs(steering_angle))

# Option 4 (Linear with deadband):
def speed_control_linear_deadband(self, max_speed, steering_angle, speed_gain):
    DEADBAND = 0.05
    
    if abs(steering_angle) < DEADBAND:
        return max_speed
    else:
        effective_angle = abs(steering_angle) - DEADBAND
        return max_speed - effective_angle * speed_gain

# Testing Protocol:
# 1. Implement all 4 formulas as methods
# 2. Add parameter to switch between them
# 3. Run 3 laps with each formula
# 4. Record lap times
# 5. Choose best performer

# Test each with these speed_gain values:
speed_gains_to_test = {
    'quadratic': [0.5, 0.7, 0.9],
    'piecewise': [0.5, 0.7, 0.9],
    'exponential': [0.3, 0.5, 0.7],
    'linear_deadband': [3.0, 4.0, 5.0]
}
```

**Testing Results Table**:
```markdown
| Formula | speed_gain | Lap Time | Crashes | Observations |
|---------|-----------|----------|---------|--------------|
| Quadratic | 0.5 |          |         |              |
| Quadratic | 0.7 |          |         |              |
| Quadratic | 0.9 |          |         |              |
| Piecewise | 0.5 |          |         |              |
| ... |  |          |         |              |
```

**Deliverable**: Advanced gap finding + optimal speed control formula

---

#### Day 12: Code Finalization (3 hours) - UNCHANGED
- [ ] Merge all features cleanly
- [ ] Add parameter validation
- [ ] Code cleanup
- [ ] Add error handling
- [ ] Write documentation

**Code Quality Checklist**:
- [ ] All parameters in config file (none hardcoded)
- [ ] Consistent variable naming
- [ ] Docstrings for all methods
- [ ] No debug print statements (use logger)
- [ ] Type hints where appropriate

---

#### Day 13-14: Optimization & Polish (3 hours) - UNCHANGED
- [ ] Profile code performance
- [ ] Optimize numpy operations
- [ ] Disable debug logging for production
- [ ] Create backup configurations
- [ ] Final testing

**Performance Checks**:
```python
# Check callback frequency
ros2 topic hz /drive  # Should be 40-50 Hz

# If too slow, optimize:
# - Use numpy vectorization
# - Pre-allocate arrays
# - Remove unnecessary calculations
# - Disable visualization in production
```

---

## 🤝 COLLABORATION POINTS

### With Mech Eng #1 (New Partnership!)

**Day 9 - Steering Smoothing Session**:
1. You prepare 5 different alpha values in config
2. Mech #1 tests each with 2 laps
3. You watch together, discuss observations
4. You implement the chosen value
5. Run final verification together

**Communication**:
- Quick 15-min sync before testing
- Live chat during tests
- 15-min debrief after

---

### With Mech Eng #2

**Day 11 - Speed Control Analysis**:
1. You implement different speed formulas
2. Mech #1 tests each
3. Mech #2 analyzes the logged data
4. Mech #2 sends you analysis results
5. You implement the recommended formula

**Data Exchange**:
- You provide: CSV log files
- They provide: Analysis recommendations
- You implement: Best configuration

---

### With Comp Eng

**Day 7**: You provide logging code → They integrate
**Day 12**: You provide final code → They deploy
**Continuous**: They help with ROS2 issues

---

## ⚡ EMERGENCY SHORTCUTS (If Time Tight!)

### Priority 1 (Must Do - 12 hours):
- ✅ Core features (smoothing, adaptive speed, quadratic) - 3h
- ✅ Basic data logging - 1h
- ✅ Steering smoothing tuning - 2h
- ✅ Speed control basics - 2h
- ✅ Code finalization - 2h
- ✅ Final testing - 2h

### Priority 2 (Nice to Have - 8 hours):
- ⚠️ Corner detection - 3h
- ⚠️ Advanced gap finding - 4h
- ⚠️ Advanced speed formulas testing - 1h

### Priority 3 (Skip if Needed - 8 hours):
- ❌ Multiple speed formula implementations
- ❌ Extensive speed_gain testing
- ❌ Code optimization beyond basics

**If really pressed for time**: Focus on Priority 1 only = 12 hours instead of 28

---

## 🎯 SUCCESS METRICS

### By Day 7:
- ✅ Steering smoothing working
- ✅ Data logging functional
- ✅ Code running smoothly

### By Day 11:
- ✅ Optimal smoothing found (with Mech #1)
- ✅ Best speed control chosen
- ✅ Corner detection working (if time allows)

### By Day 14:
- ✅ Clean, production-ready code
- ✅ All features integrated
- ✅ Backup configs prepared
- ✅ Team confident in the system

---

## 💪 YOU CAN HANDLE THIS!

**28 hours sounds like a lot, but**:
- You have the strongest coding skills
- Many tasks are parameter tuning (not new code)
- You're working with Mech #1 on steering (collaborative!)
- Speed control is mostly testing existing formulas
- Data logging is straightforward

**Break it down**:
- Week 1: 10 hours (5h per session)
- Week 2: 18 hours (can split across multiple sessions)
- ~2 hours per day average
- Can do more on weekends, less on midterm days

**You're the key player now** - the algorithm's success depends on you! 🚀

---

## 📞 When to Ask for Help

**Ask Comp Eng**:
- ROS2 integration issues
- Launch file problems
- System errors

**Ask Mech #1**:
- Does steering look smooth?
- Is speed behavior correct?
- Testing observations

**Ask Mech #2**:
- What does the data show?
- Which formula performed best?
- Statistical analysis

**Ask Team**:
- Major design decisions
- Priority trade-offs
- Time allocation

---

Good luck! You're now the algorithm architect! 💻🏎️
