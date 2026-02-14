# MECHANICAL ENGINEER #2 - UPDATED TASK LIST (4-PERSON TEAM)
## Role: Data Analysis & Control Performance Analysis

**Updated Time**: 23 hours (+3h from EE departure)

---

## ⚠️ WHAT CHANGED

You're now responsible for:
- ✅ **Steering smoothing data analysis** (was EE's task)
- ✅ **Speed control data analysis** (was EE's task)
- ✅ **Control system performance metrics** (was EE's task)

**Why you?** You're already analyzing all the test data - now you're looking at control behavior too!

**Good news**: You'll use the same data analysis skills, just applied to steering/speed.

---

## 📅 UPDATED TASK BREAKDOWN

### Week 1 Tasks (6 hours total) - MOSTLY UNCHANGED

#### Day 1-2: Testing Framework Setup (3 hours)
- [ ] Create master spreadsheet
- [ ] Design data collection forms
- [ ] Set up tracking system
- [ ] Prepare visualization templates

**Same as before** - see original task list

---

#### Day 6-7: Baseline Data Analysis (3 hours)
- [ ] Process Mech #1's baseline data
- [ ] Calculate statistics
- [ ] Identify weakest sections
- [ ] Generate recommendations

**Same as before** - see original task list

---

### Week 2 Tasks (17 hours total) - ADDITIONS

#### Day 8-9: Speed Data + Steering Smoothing Analysis (5 hours) - ADDED 2h

**Original: Speed Data Analysis (3h)**:
Same as before - analyze lap time vs max_speed

**NEW: Steering Smoothing Analysis (2h)**:

After CS Student and Mech #1 run smoothing tests, analyze the results:

**Analysis Tasks**:

1. **Compile Test Data**:
```markdown
## Steering Smoothing Test Results

| Alpha | Lap 1 Time | Lap 2 Time | Avg Time | Oscillations | Rating |
|-------|-----------|-----------|----------|--------------|--------|
| 0.1   |           |           |          |              |        |
| 0.2   |           |           |          |              |        |
| 0.3   |           |           |          |              |        |
| 0.4   |           |           |          |              |        |
| 0.5   |           |           |          |              |        |
```

2. **Calculate Metrics**:
- Lap time consistency (std deviation)
- Time penalty vs baseline (alpha=0)
- Oscillation frequency (from Mech #1's observations)

3. **Create Visualization** (Excel/Sheets):
- Chart: Alpha value vs Lap Time
- Chart: Alpha value vs Oscillation severity
- Note the "sweet spot"

4. **Quantitative Analysis**:
```python
# Simple Python analysis (optional):
import pandas as pd
import numpy as np

data = {
    'alpha': [0.1, 0.2, 0.3, 0.4, 0.5],
    'lap_time': [45.2, 44.8, 44.5, 45.1, 45.8],
    'oscillations': [5, 3, 1, 0, 0]  # severity 0-5
}
df = pd.DataFrame(data)

# Find minimum time
best_time = df.loc[df['lap_time'].idxmin()]
print(f"Fastest: Alpha={best_time['alpha']}, Time={best_time['lap_time']}")

# Find minimum oscillation
best_smooth = df.loc[df['oscillations'].idxmin()]
print(f"Smoothest: Alpha={best_smooth['alpha']}")

# Recommend compromise
# Usually want oscillations ≤ 1 with best lap time
recommended = df[(df['oscillations'] <= 1)].loc[df['lap_time'].idxmin()]
print(f"Recommended: Alpha={recommended['alpha']}")
```

5. **Make Recommendation**:
```markdown
## Steering Smoothing Recommendation

### Data Summary:
- Tested range: Alpha 0.1 to 0.5
- Best lap time: _____ at alpha = _____
- Smoothest behavior: alpha = _____

### Trade-off Analysis:
- Low alpha (0.1-0.2): Fast but oscillates
- Medium alpha (0.3): Balanced
- High alpha (0.4-0.5): Very smooth but slower response

### Recommendation: Alpha = _____

**Rationale**:
_________________________________

### Alternative (if primary has issues):
Alpha = _____
```

**Deliverable**: Smoothing analysis report with recommendation

---

#### Day 10-11: Steering Testing + Speed Control Analysis (4 hours) - ADDED 1h

**Original: Steering Behavior Analysis (3h)**:
Same as before - analyze steering_gain tests

**NEW: Speed Control Analysis (1h)**:

Analyze the relationship between steering and speed:

**Analysis Questions to Answer**:

1. **Is the speed formula working correctly?**
```markdown
## Speed Control Analysis

### Expected Behavior:
- Straight (steering < 0.1): Speed = max_speed
- Gentle turn (0.1-0.2): Speed slightly reduced
- Sharp turn (>0.2): Speed significantly reduced

### Actual Behavior (from logs):
- Straight: Average speed = _____ (target: _____)
- Gentle turn: Average speed = _____ (target: _____)
- Sharp turn: Average speed = _____ (target: _____)

### Gap Analysis:
- Underutilizing speed? ☐ Yes ☐ No
- Too aggressive in corners? ☐ Yes ☐ No
- Formula effectiveness: ☐ Good ☐ Needs tuning
```

2. **Speed Profile by Track Section**:

Use Mech #1's speed observations + CSV logs:

```markdown
| Section | Target | Actual | Efficiency | Issue? |
|---------|--------|--------|-----------|--------|
| Straight 1 | 7.0 | 6.5 | 93% | Not reaching max |
| Corner 1 | 4.5 | 4.2 | 93% | OK |
| ...     |     |     |       |        |

### Overall Speed Efficiency: ____%

### Recommendations:
- Max speed too high/low? _____
- Speed gain needs adjustment? _____
- Specific sections to optimize: _____
```

3. **Speed vs Steering Correlation**:

If you have CSV logs from CS Student:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load data
data = pd.read_csv('race_log_XXXXXX.csv')

# Plot speed vs steering
plt.figure(figsize=(10, 6))
plt.scatter(abs(data['steering_angle']), data['speed'], alpha=0.5)
plt.xlabel('Abs(Steering Angle)')
plt.ylabel('Speed (m/s)')
plt.title('Speed vs Steering Relationship')
plt.grid(True)
plt.savefig('speed_vs_steering.png')

# Check if relationship makes sense
# Should see: speed decreases as steering increases
```

4. **Make Recommendation**:
```markdown
## Speed Control Recommendation

### Current Formula Performance:
- Formula type: Quadratic
- speed_gain: _____
- Performance: ☐ Excellent ☐ Good ☐ Needs improvement

### Issues Found:
1. _________________________________
2. _________________________________

### Recommendations:
- Keep current formula: ☐ Yes ☐ No
- Adjust speed_gain to: _____
- Try alternative formula: ☐ Yes ☐ No

### Expected Improvement:
If recommendations implemented: _____ seconds faster
```

**Deliverable**: Speed control analysis with recommendations

---

#### Day 12-14: Same as Before
- Day 12: Configuration comparison (4h)
- Day 13: Endurance data analysis (3h)
- Day 14: Final documentation (2h)

**No changes** - see original task list

---

## 🤝 NEW COLLABORATION POINTS

### With CS Student (New!)

**Day 9**:
- They provide: CSV logs from smoothing tests
- You provide: Analysis and recommendation
- Together: Choose optimal alpha value

**Day 11**:
- They provide: Speed control test results
- You provide: Speed efficiency analysis
- Together: Decide if formula needs changing

**Communication**:
- Request CSV files after tests
- Send analysis within 24 hours
- Quick call to discuss findings

---

### With Mech Eng #1 (Same as Before)

- They test → You analyze
- They observe behavior → You quantify it
- They provide notes → You add numbers

**Day 9 collaboration**:
- They describe oscillations qualitatively
- You quantify the impact on lap time
- Together recommend optimal value

---

## 📊 NEW ANALYSIS SKILLS NEEDED

### Steering Behavior Metrics

**What to Calculate**:
1. **Consistency**: Lap-to-lap time variation
2. **Efficiency**: Time cost of smoothing
3. **Stability**: Oscillation frequency/severity

**How to Measure**:
```markdown
### Consistency Score
- Std deviation of lap times
- Lower is better
- Target: < 0.5 seconds

### Efficiency Score
- Smoothing penalty = (smooth_time - baseline_time)
- Target: < 2 seconds penalty

### Stability Score  
- Oscillation rating from Mech #1
- Target: ≤ 1 (slight) on 0-5 scale
```

---

### Speed Control Metrics

**What to Calculate**:
1. **Speed Utilization**: % of max_speed achieved on straights
2. **Corner Efficiency**: Speed maintained through turns
3. **Formula Effectiveness**: Does actual match expected?

**Example Calculation**:
```
Speed Utilization = (Average Straight Speed / max_speed) × 100%

Target: > 90%
If < 90%: Car not reaching full speed potential
If > 95%: Very good utilization
```

---

## 🔧 TOOLS & TECHNIQUES

### Excel/Google Sheets (Primary Tool)

**Formulas to Use**:
```
Average: =AVERAGE(B2:B10)
Std Dev: =STDEV(B2:B10)
Min: =MIN(B2:B10)
Max: =MAX(B2:B10)
Correlation: =CORREL(A2:A10, B2:B10)
```

**Charts to Create**:
- Line chart: Parameter vs Lap Time
- Scatter plot: Steering vs Speed
- Bar chart: Config comparison

---

### Python (Optional but Helpful)

**Basic Analysis Script**:
```python
import pandas as pd
import numpy as np

# Load test results
data = pd.read_csv('test_results.csv')

# Calculate statistics
print(f"Mean lap time: {data['lap_time'].mean():.2f}s")
print(f"Std deviation: {data['lap_time'].std():.2f}s")
print(f"Best lap: {data['lap_time'].min():.2f}s")
print(f"Completion rate: {(data['completed']=='Yes').mean()*100:.1f}%")

# Find optimal parameter
best_config = data.loc[data['lap_time'].idxmin()]
print(f"\nBest configuration:")
print(best_config)
```

---

## ⚡ EMERGENCY SHORTCUTS (If Time Tight!)

### Minimum Analysis (15 hours):

**Day 1-2**: Basic spreadsheet (2h)
**Day 6-7**: Baseline stats only (2h)
**Day 8-9**: Speed analysis (2h), skip detailed smoothing analysis
**Day 10-11**: Basic steering analysis (2h), skip speed control depth
**Day 12**: Quick config comparison (3h)
**Day 13**: Basic reliability check (2h)
**Day 14**: 1-page reference doc (2h)

**Total: 15 hours** (vs 23 hours full plan)

**What to skip**:
- Fancy visualizations
- Python analysis (use Excel only)
- Deep statistical analysis
- Multiple alternative recommendations

**What NOT to skip**:
- Basic statistics (mean, std dev)
- Config comparisons
- Final recommendations
- Quick reference guide

---

## 🎯 SUCCESS METRICS

### By Day 9:
- ✅ Speed optimization data analyzed
- ✅ Steering smoothing recommendation made
- ✅ Clear data-driven guidance to team

### By Day 11:
- ✅ Steering behavior quantified
- ✅ Speed control effectiveness assessed
- ✅ Improvement recommendations provided

### By Day 14:
- ✅ All test data compiled
- ✅ Final configuration justified with data
- ✅ Complete documentation ready
- ✅ Team has confidence in chosen config

---

## 💪 YOU CAN HANDLE THIS!

**The extra 3 hours breaks down as**:
- 2h: Steering smoothing analysis (Day 9)
- 1h: Speed control analysis (Day 11)

**These are the SAME analysis skills you're already using, just applied to different data!**

**Think of it as**:
- Before: Analyze lap times and crashes
- Now: Analyze lap times, crashes, steering, AND speed

**Your data skills are crucial for optimal tuning!** 📊🎯

---

## 📋 ANALYSIS TEMPLATES

### Quick Analysis Template
```markdown
## Parameter: _____

### Data:
[Table of results]

### Statistics:
- Mean: _____
- Best: _____
- Worst: _____
- Std Dev: _____

### Findings:
- Trend observed: _____
- Optimal value: _____
- Confidence: ☐ High ☐ Medium ☐ Low

### Recommendation:
Use _____ because _____
```

### Quick Comparison Template
```markdown
## Config A vs Config B

|  Metric | Config A | Config B | Winner |
|---------|----------|----------|--------|
| Avg Time |          |          |        |
| Best Time |         |          |        |
| Crashes |          |          |        |
| Consistency |       |          |        |

**Recommendation**: Config _____ because _____
```

---

## 📞 When to Ask for Help

**Ask CS Student**:
- Need CSV log files
- Clarify what data means
- Understand speed formula

**Ask Mech #1**:
- Clarify observations
- What did "moderate oscillation" feel like?
- Get more detailed notes

**Ask Comp Eng**:
- Help with data extraction
- ROS2 logging issues

**Ask Team**:
- Major recommendation decisions
- Trade-off discussions

---

Remember: **Numbers don't lie!** Your analysis gives the team confidence! 📊✨
