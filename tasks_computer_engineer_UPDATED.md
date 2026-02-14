# COMPUTER ENGINEER - UPDATED TASK LIST (4-PERSON TEAM)
## Role: System Integration & Infrastructure

**Updated Time**: 18 hours (+2h from EE departure)

---

## ⚠️ WHAT CHANGED

You're now responsible for:
- ✅ **Basic LIDAR sensor verification** (was EE's task)
- ✅ **Final system calibration check** (was EE's task)

**Why you?** You're already managing the system - basic health checks fit naturally!

**Good news**: These are simple verification tasks, not deep technical analysis.

---

## 📅 UPDATED TASK BREAKDOWN

### Week 1 Tasks (9 hours total) - SMALL ADDITION

#### Day 1-2: Setup + Sensor Check (3 hours) - ADDED 30min

**Original: Environment Setup (2.5h)**:
- [ ] Install F1TENTH simulator
- [ ] Create ROS2 workspace
- [ ] Test base gap_finder code
- [ ] Document setup issues
- [ ] Create team repo (optional)

**NEW: Basic Sensor Verification (30min)**:

After getting simulator running, do a quick LIDAR health check:

```bash
# Terminal 1: Launch simulator
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: Check sensor is publishing
ros2 topic list | grep scan
# Should see: /scan

# Check publishing frequency
ros2 topic hz /scan
# Should see: ~40-50 Hz

# Echo one message to check data
ros2 topic echo /scan --once
```

**Check for**:
- ✅ Topic is publishing
- ✅ Frequency is reasonable (30-50 Hz)
- ✅ Range array has data (not all zeros)
- ✅ No excessive NaN or Inf values

**Document in sensor_check.txt**:
```markdown
# LIDAR Sensor Verification

Date: _____
Simulator Version: _____

## Tests Performed:
- [ ] /scan topic exists
- [ ] Publishing frequency: _____ Hz (target: 40-50)
- [ ] Range array length: _____ points
- [ ] Valid data: ☐ Yes ☐ No
- [ ] Issues found: _____

## Sensor Specs (from message):
- angle_min: _____
- angle_max: _____
- angle_increment: _____
- range_min: _____
- range_max: _____

## Status: ☐ PASS ☐ FAIL

Notes: _____________________
```

**If anything looks wrong**:
1. Check simulator installation
2. Restart simulator
3. Ask team if anyone else seeing issues
4. Document for reference

**Deliverable**: Working environment + sensor verification doc

---

#### Day 6-7: Integration Foundation (3 hours) - UNCHANGED

Same as before:
- Integrate improved code
- Set up parameter loading
- Create launch file
- Test parameter hot-swapping

---

### Week 2 Tasks (9 hours total) - SMALL ADDITION

#### Day 8-9: Testing Support (2 hours) - UNCHANGED

Monitor tests, fix bugs, help team

---

#### Day 10-11: Feature Integration (3 hours) - UNCHANGED

Integrate CS Student's new features

---

#### Day 12: Configuration Management (3 hours) - UNCHANGED

Create multiple config files

---

#### Day 13: Endurance Monitoring (1 hour) - UNCHANGED

Monitor 10-lap test

---

#### Day 14: Final Prep + Calibration Check (2 hours) - ADDED 30min

**Original: Race Day Prep (1.5h)**:
- Create quick-start script
- Test emergency recovery
- Final system check

**NEW: Final Calibration Verification (30min)**:

Before declaring race-ready, verify all systems:

**Calibration Checklist**:
```markdown
# Final System Calibration Check

Date: _____
Configuration: [Race Config Name]

## 1. Parameter Verification
- [ ] Config file loads correctly
- [ ] All parameters have expected values
- [ ] No hardcoded values in code

Check key parameters match config:
```bash
ros2 param list /gap_follower
ros2 param get /gap_follower max_speed
ros2 param get /gap_follower steering_gain
# etc...
```

## 2. Sensor Health
- [ ] LIDAR still publishing at 40-50 Hz
- [ ] No error messages in console
- [ ] Visualization working in RViz

## 3. Control Output
- [ ] /drive topic publishing
- [ ] Steering commands reasonable (-0.34 to +0.34)
- [ ] Speed commands reasonable (min_speed to max_speed)

Test:
```bash
ros2 topic hz /drive  # Should match LIDAR frequency
ros2 topic echo /drive  # Check values make sense
```

## 4. Performance Check
- [ ] Run 1 quick test lap
- [ ] No crashes
- [ ] Lap time within expected range

## 5. Emergency Procedures
- [ ] Can restart node quickly (<30 seconds)
- [ ] Can switch configs (<1 minute)
- [ ] Backup config verified working

## Status: ☐ CALIBRATED ☐ NEEDS ADJUSTMENT

Issues Found: _____________________
Actions Taken: _____________________
```

**If issues found**:
1. Work with CS Student to fix
2. Re-verify after fixes
3. Document all changes

**Deliverable**: Calibration report + race-ready system

---

## 🔧 SIMPLE DIAGNOSTIC COMMANDS

### Quick Health Checks (Use These Anytime)

**1. Check System Status**:
```bash
# List all nodes
ros2 node list

# Check specific node info
ros2 node info /gap_follower

# List all topics
ros2 topic list
```

**2. Monitor Performance**:
```bash
# Check frequency of any topic
ros2 topic hz /scan
ros2 topic hz /drive

# Watch messages in real-time
ros2 topic echo /drive

# Check parameter values
ros2 param list /gap_follower
ros2 param get /gap_follower max_speed
```

**3. Troubleshooting**:
```bash
# If node not responding, restart:
Ctrl+C  # Kill the node
ros2 launch your_package gap_finder_launch.py  # Restart

# If parameters not loading:
ros2 param list /gap_follower  # Check if params exist
# If missing, check config file path in launch file

# If sensor issues:
ros2 topic info /scan  # Check publisher
ros2 topic echo /scan --once  # Verify data
```

---

## 🎯 YOUR ROLE CLARITY

### What You ARE Responsible For:
- ✅ System starts up correctly
- ✅ All components talk to each other (ROS2 integration)
- ✅ Parameters load from config files
- ✅ Basic health checks pass
- ✅ Quick bug fixes and restarts

### What You ARE NOT Responsible For:
- ❌ Deep algorithm development (CS Student's job)
- ❌ Parameter optimization (Mech team's job)
- ❌ Data analysis (Mech #2's job)
- ❌ Advanced sensor processing

**Think of yourself as**: The infrastructure keeper, not the algorithm expert

---

## ⚡ EMERGENCY SHORTCUTS (If Time Tight!)

### Minimum Work (12 hours):

**Day 1-2**: Basic setup (2h)
- Skip detailed sensor docs
- Just verify it works

**Day 6-7**: Quick integration (2h)
- Get code running
- Skip extensive testing

**Day 8-11**: Minimal support (3h)
- Only help when asked
- Focus on blocking issues

**Day 12**: Basic configs (2h)
- Create just 2 configs (race + backup)

**Day 13-14**: Final check (3h)
- Quick verification
- Race day script

**Total: 12 hours** (vs 18 hours full plan)

---

## 🤝 COLLABORATION NOTES

### You're the Support Hub

**Everyone comes to you for**:
- "It won't start"
- "Parameters aren't loading"
- "ROS2 error messages"
- "How do I restart the system?"

**Keep calm and troubleshoot!**

### Your Value

Even though you don't write the main algorithm:
- ✅ You enable everyone else to work
- ✅ You keep the system stable
- ✅ You handle integration complexity
- ✅ You're the race day operator

**Without you, the team can't test!**

---

## 📋 SIMPLE TROUBLESHOOTING GUIDE

### Problem: Simulator won't start
```bash
# Check if already running:
ps aux | grep gym_bridge

# Kill if stuck:
killall -9 gym_bridge

# Restart:
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Problem: Node won't start
```bash
# Check Python syntax:
python3 gap_finder_improved_v1.py
# (Fix any errors shown)

# Check file permissions:
ls -l gap_finder_improved_v1.py
# Should be executable

# Make executable if needed:
chmod +x gap_finder_improved_v1.py
```

### Problem: Parameters not loading
```bash
# Check config file exists:
ls -l config/config_nurburgring.yaml

# Check launch file has correct path:
cat launch/gap_finder_launch.py
# Verify config path is correct

# Test loading manually:
ros2 param load /gap_follower config/config_nurburgring.yaml
```

### Problem: Poor performance
```bash
# Check CPU usage:
top
# gap_follower should use reasonable CPU (< 50%)

# Check message frequency:
ros2 topic hz /scan
ros2 topic hz /drive
# Both should be ~40-50 Hz

# If too slow:
# - Disable debug logging
# - Disable visualizations
# - Check for infinite loops in code
```

---

## ✅ SUCCESS METRICS

### By Day 2:
- ✅ Simulator running
- ✅ Base code tested
- ✅ Sensor verified working

### By Day 7:
- ✅ Improved code integrated
- ✅ Parameter system working
- ✅ Team can test easily

### By Day 14:
- ✅ All systems verified
- ✅ Race config loaded
- ✅ Quick-restart procedures ready
- ✅ Confident in system stability

---

## 💪 YOU'VE GOT THIS!

**The extra 2 hours is just**:
- 30 min: Initial sensor check (one-time)
- 30 min: Final calibration check (one-time)
- 1 hour: Buffer for unexpected integration issues

**That's it!** The rest is what you were already doing.

**Your job is straightforward**:
1. Make it run
2. Keep it running
3. Help others when it doesn't run

**You're the foundation the team builds on!** 🛠️🏗️

---

## 📞 When to Ask for Help

**Ask CS Student**:
- Code is broken (syntax errors)
- Algorithm behaving strangely
- Need features implemented

**Ask Mech #1**:
- Does the system behavior match expectations?
- Is testing going smoothly?

**Ask Mech #2**:
- Need data extraction help
- Performance metrics unclear

**Ask Team**:
- Major system decisions
- Config file structure
- Race day procedures

---

Remember: **Keep it simple, keep it running, keep the team moving!** 🔧✅
