# 4-PERSON TEAM TASK REDISTRIBUTION
## After Electronics Engineer Departure

---

## 📊 NEW TEAM COMPOSITION

| Role | Member | Original Hours | New Hours | Added Tasks |
|------|--------|----------------|-----------|-------------|
| **Integration Lead** | Computer Engineer | 16h | 18h | +2h (sensor checks, calibration) |
| **Algorithm Lead** | CS Student | 20h | 28h | +8h (data logging, control tuning) |
| **Testing Lead** | Mech Engineer #1 | 22h | 25h | +3h (steering analysis) |
| **Data Analyst** | Mech Engineer #2 | 20h | 23h | +3h (control data analysis) |

**Total Team Effort**: 94 hours (was 97 with 5 people)
**Per Person**: 18-28 hours (~2 hours/day avg)

---

## 🔄 REDISTRIBUTED ELECTRONICS TASKS

### What We're Keeping (Critical)
✅ Data logging implementation → **CS Student**
✅ Steering smoothing tuning → **CS Student + Mech #1**
✅ Speed control optimization → **CS Student**
✅ Sensor verification → **Comp Eng** (basic)
✅ Performance data analysis → **Mech #2**

### What We're Simplifying
⚠️ Advanced filtering → Make optional, use built-in smoothing
⚠️ Deep sensor analysis → Basic checks only
⚠️ Kalman filters → Skip (too complex for time available)

### What We're Skipping
❌ Real-time visualization tools
❌ Advanced control theory implementation
❌ Detailed signal processing analysis

---

## 👥 UPDATED INDIVIDUAL RESPONSIBILITIES

### Computer Engineer - "The Integrator"
**New Time**: 18 hours (+2h)

**Added Tasks**:
- **Day 1-2**: Basic LIDAR sensor check (1h)
  - Verify scan topic publishing
  - Check data quality (no NaN/Inf values)
  - Document sensor specs
  
- **Day 14**: Final system calibration (1h)
  - Verify all parameters loaded correctly
  - Quick sensor health check
  - System performance verification

**Everything else stays the same** - see original task list

---

### CS Student - "The Algorithm Developer"
**New Time**: 28 hours (+8h)

**Added Tasks**:

**Day 7**: Data Logging (3h)
```python
# Add simple CSV logger to gap_finder
import csv
from datetime import datetime

class PerformanceLogger:
    def __init__(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'race_log_{timestamp}.csv'
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'steering', 'speed', 
                             'gap_width', 'max_dist'])
    
    def log(self, timestamp, steering, speed, gap_width, max_dist):
        self.writer.writerow([timestamp, steering, speed, 
                             gap_width, max_dist])
        self.file.flush()
    
    def close(self):
        self.file.close()

# Integrate into lidar_callback
self.logger.log(time, steering, speed, gap_width, max_dist)
```

**Day 9**: Steering Smoothing Tuning (2h)
- Test different smoothing alpha values: [0.1, 0.2, 0.3, 0.4, 0.5]
- Record oscillation behavior for each
- Choose best value with Mech #1's input

**Day 11**: Speed Control Optimization (3h)
- Test different speed formulas:
  ```python
  # Option 1: Quadratic (current)
  speed = max_speed - (abs(steering) ** 2) * speed_gain
  
  # Option 2: Piecewise
  if abs(steering) < 0.1:
      speed = max_speed
  else:
      speed = max_speed - (abs(steering) ** 2) * speed_gain
  
  # Option 3: Exponential
  speed = max_speed * np.exp(-speed_gain * abs(steering))
  ```
- Compare lap times and choose best

**Everything else stays the same** - see original task list

---

### Mechanical Engineer #1 - "The Tester"
**New Time**: 25 hours (+3h)

**Added Tasks**:

**Day 9**: Steering Behavior Analysis (2h)
- During speed tests, specifically observe:
  - Does car oscillate/wobble?
  - Frequency of oscillations (if any)
  - Smoothness of steering transitions
- Work with CS Student to tune smoothing parameter
- Document which alpha value gives smoothest path

**Day 11**: Speed Profile Documentation (1h)
- Record actual speeds achieved in different track sections
- Note: Straight speed, corner entry, corner exit
- Compare to target max_speed
- Report findings to CS Student for optimization

**Everything else stays the same** - see original task list

---

### Mechanical Engineer #2 - "The Data Analyst"
**New Time**: 23 hours (+3h)

**Added Tasks**:

**Day 9**: Steering Smoothing Analysis (2h)
- Analyze steering data from tests
- Calculate metrics:
  - Steering oscillation frequency
  - Steering angle variance
  - Path smoothness score
- Create comparison chart for different alpha values
- Recommend optimal smoothing parameter

**Day 11**: Speed Control Analysis (1h)
- Analyze speed vs steering relationship
- Check if speed formula is working as intended
- Calculate average speed in different scenarios
- Recommend speed_gain adjustments if needed

**Everything else stays the same** - see original task list

---

## 📅 UPDATED TIMELINE WITH NEW RESPONSIBILITIES

### Week 1 (Days 1-7)

**Day 1-2**: Setup & Analysis
- **Comp Eng**: Setup + **SENSOR CHECK** (3h total)
- **CS Student**: Code analysis (3h)
- **Mech #1**: Track analysis (3h)
- **Mech #2**: Testing framework (3h)

**Day 3-5**: CNY Break 🧧

**Day 6-7**: First Improvements
- **Comp Eng**: Integration (3h)
- **CS Student**: Core features + **DATA LOGGING** (5h)
- **Mech #1**: Baseline testing (4h)
- **Mech #2**: Data analysis (3h)

---

### Week 2 (Days 8-14)

**Day 8-9**: Parameter Sweep Part 1
- **CS Student**: Corner detection + **SMOOTHING TUNING** (5h)
- **Mech #1**: Speed testing + **STEERING ANALYSIS** (6h)
- **Mech #2**: Data analysis + **SMOOTHING ANALYSIS** (5h)
- **Comp Eng**: Test support (2h)

**Day 10-11**: Parameter Sweep Part 2
- **CS Student**: Gap finding + **SPEED OPTIMIZATION** (7h)
- **Mech #1**: Steering testing + **SPEED PROFILING** (5h)
- **Mech #2**: Analysis + **SPEED ANALYSIS** (4h)
- **Comp Eng**: Integration (3h)

**Day 12**: Integration
- All team members: Same as before (3h each)

**Day 13**: Endurance Testing
- All team members: Same as before (2-3h each)

**Day 14**: Final Prep
- **Comp Eng**: Integration + **FINAL CALIBRATION** (2h)
- **CS Student**: Optimization (2h)
- **Mech #1**: Race checklist (1h)
- **Mech #2**: Documentation (2h)

---

## 🎯 SIMPLIFIED APPROACH (Due to Smaller Team)

### What We're Focusing On
1. ✅ **Steering smoothing** (critical for stability)
2. ✅ **Speed control** (critical for performance)
3. ✅ **Data logging** (needed for analysis)
4. ✅ **Basic sensor checks** (safety)

### What We're Skipping
1. ❌ Advanced Kalman filtering
2. ❌ Complex signal processing
3. ❌ Real-time visualization
4. ❌ Acceleration limiting
5. ❌ Moving average filters

### Why This Still Works
- **Steering smoothing** was the #1 priority anyway
- **Speed control** is mostly parameter tuning
- **Built-in features** handle basic filtering
- **Simpler = fewer bugs** = more reliable

---

## 💡 TEAM COORDINATION ADJUSTMENTS

### Collaboration Points

**CS Student ↔ Mech #1** (New Partnership):
- Day 9: Tune steering smoothing together
  - Mech #1 observes behavior
  - CS Student adjusts alpha parameter
  - Iterate until smooth

**CS Student ↔ Mech #2** (Enhanced):
- Mech #2 analyzes logged data
- CS Student implements recommendations
- Tighter feedback loop

**Comp Eng** (Still the glue):
- Less technical depth needed
- More focus on integration
- Basic health checks only

---

## ⚡ EMERGENCY SHORTCUTS (Even More Important Now!)

If time gets tight, **skip these first**:
1. Advanced gap finding (use basic max-distance)
2. Corner detection (nice to have)
3. Multiple configuration testing (pick one good config)
4. Detailed logging (basic timing is enough)

**Minimum Viable Product** (if really pressed):
- Day 1-7: Get improved code running
- Day 8-9: Find optimal max_speed
- Day 10-11: Tune steering_gain
- Day 12: Pick best combo, test 5 laps
- Day 13-14: If it works, you're done!

**Total time if using shortcuts**: ~12 hours per person

---

## 📊 WORK DISTRIBUTION FAIRNESS

### CS Student Gets Most New Work (Why?)
- ✅ Has strongest coding skills
- ✅ Already familiar with the codebase
- ✅ Control tuning is programming work
- ✅ Can work more independently
- ✅ 28h is still manageable (~2h/day)

### Mech Engineers Share Analysis
- ✅ Play to their strengths (observation, data)
- ✅ Don't need to write code
- ✅ Work together on steering analysis
- ✅ Balanced workload (23-25h each)

### Comp Eng Stays Light
- ✅ Integration is already demanding
- ✅ Need to be available for everyone
- ✅ Basic checks don't require deep expertise
- ✅ 18h keeps them responsive

---

## ✅ UPDATED SUCCESS METRICS

### Must Achieve (4 people can do this)
- ✅ Car completes laps reliably
- ✅ Lap time < 50 seconds
- ✅ Steering smoothing working
- ✅ Data logging functional

### Good Target (realistic for 4)
- ✅ Lap time < 45 seconds
- ✅ 90%+ completion rate
- ✅ Optimized speed control
- ✅ Well-documented process

### Stretch Goal (if everything goes well)
- ✅ Lap time < 42 seconds
- ✅ 100% reliability
- ✅ Advanced features working
- ✅ Competitive performance

---

## 🆘 IF SOMEONE ELSE QUITS

**Priority Rankings**:

**Most Critical** (Don't lose these):
1. CS Student - algorithm development
2. Mech #1 - testing

**Important** (Would hurt but survivable):
3. Comp Eng - CS Student could take over
4. Mech #2 - Mech #1 could do basic analysis

**Minimum Viable Team**: 2 people
- CS Student: Code everything
- Mech #1: Test everything
- 40+ hours each, but doable

---

## 🎯 POSITIVE SPIN

### Advantages of 4 People
- ✅ Fewer coordination meetings needed
- ✅ Clearer decision making
- ✅ More focused work streams
- ✅ Less chance of conflicting changes

### What Doesn't Change
- ✅ Algorithm is still improved from base
- ✅ Testing methodology still systematic
- ✅ Timeline still achievable
- ✅ Goal still to learn and compete

---

## 📞 UPDATED CONTACT GUIDE

| Question | Ask |
|----------|-----|
| "ROS2 won't start" | Comp Eng |
| "How does this code work?" | CS Student |
| "Code isn't working" | CS Student |
| "Which parameter to test?" | Mech #1 |
| "What do numbers mean?" | Mech #2 |
| "Steering is shaky" | CS Student + Mech #1 |
| "Speed seems wrong" | CS Student + Mech #2 |
| "Strategy question" | Team meeting |

---

## 🏁 YOU'VE STILL GOT THIS!

**4 people is plenty for this hackathon!**

Other teams might have:
- 1-2 people doing everything
- 5+ people with poor coordination
- No clear plan

**You still have**:
- ✅ Balanced team with clear roles
- ✅ Detailed task breakdown
- ✅ Improved starting algorithm
- ✅ Systematic testing approach
- ✅ Realistic timeline

**The plan is adjusted, not broken!** 💪

---

Let me create the updated individual task files next...
