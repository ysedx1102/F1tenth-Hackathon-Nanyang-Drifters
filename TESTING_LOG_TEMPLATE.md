# NÜRBURGRING PARAMETER TESTING LOG

## Test Session Information
- **Date**: _____________
- **Track**: Nürburgring
- **Simulator Version**: _____________
- **Goal**: Find optimal parameters for race

---

## BASELINE TEST

### Test #0 - Original Parameters
**Date/Time**: _____________

**Parameters:**
- max_speed: 5.0
- steering_gain: 0.4
- speed_gain: 1.0
- lookahead_distance: 5.0
- obstacle_bubble_radius: 0.15
- smooth_steering: false
- adaptive_speed: false

**Results:**
- ☐ Completed Lap  ☐ Crashed
- Lap Time: _______ seconds
- Crash Location (if applicable): _____________
- Observations:
  - Speed on straights: _______
  - Behavior in corners: _______
  - Oscillation/wobbling: ☐ Yes ☐ No
  - Got stuck: ☐ Yes ☐ No

**Notes:**
_____________________________________________
_____________________________________________

---

## SPEED OPTIMIZATION TESTS

### Test #1 - Moderate Speed Increase
**Date/Time**: _____________

**Changed Parameters:**
- max_speed: 6.0 (was 5.0)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Better than baseline? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

### Test #2 - Higher Speed
**Date/Time**: _____________

**Changed Parameters:**
- max_speed: 7.0 (was 6.0)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Better than previous? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

### Test #3 - Aggressive Speed
**Date/Time**: _____________

**Changed Parameters:**
- max_speed: 8.0 (was 7.0)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Too risky? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

## STEERING BEHAVIOR TESTS

### Test #4 - Aggressive Steering
**Date/Time**: _____________

**Changed Parameters:**
- steering_gain: 0.6 (was 0.5)
- speed_gain: 0.7 (was 1.0)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Cornering improved? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

### Test #5 - Conservative Steering
**Date/Time**: _____________

**Changed Parameters:**
- steering_gain: 0.4 (was 0.5)
- speed_gain: 1.2 (was 1.0)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- More stable? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

## SAFETY vs SPEED TESTS

### Test #6 - Tighter Safety Margins
**Date/Time**: _____________

**Changed Parameters:**
- obstacle_bubble_radius: 0.10 (was 0.15)
- disparity_bubble_radius: 0.10 (was 0.15)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Too risky? ☐ Yes ☐ No
- Number of close calls: _______

**Notes:**
_____________________________________________

---

### Test #7 - Wider Safety Margins
**Date/Time**: _____________

**Changed Parameters:**
- obstacle_bubble_radius: 0.18 (was 0.15)
- disparity_bubble_radius: 0.18 (was 0.15)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Too conservative? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

## ADVANCED FEATURE TESTS

### Test #8 - Steering Smoothing
**Date/Time**: _____________

**Changed Parameters:**
- smooth_steering: true (was false)
- steering_smoothing_alpha: 0.3

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Reduced oscillation? ☐ Yes ☐ No
- Path smoothness (1-10): _______

**Notes:**
_____________________________________________

---

### Test #9 - Adaptive Speed
**Date/Time**: _____________

**Changed Parameters:**
- adaptive_speed: true (was false)
- gap_width_bonus: 0.5

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Faster on straights? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

### Test #10 - Vision Range Tuning
**Date/Time**: _____________

**Changed Parameters:**
- lookahead_distance: 7.0 (was 5.0)
- field_of_vision: 2.09 (was 1.57, ~120° vs 90°)

**Results:**
- ☐ Completed  ☐ Crashed at: _______
- Lap Time: _______ (Δ: _______)
- Better anticipation? ☐ Yes ☐ No

**Notes:**
_____________________________________________

---

## COMBINATION TESTS

### Test #11 - Best Combined Settings (Version 1)
**Date/Time**: _____________

**Description**: Combining the best parameters from individual tests

**Parameters:**
- max_speed: _______
- steering_gain: _______
- speed_gain: _______
- lookahead_distance: _______
- obstacle_bubble_radius: _______
- disparity_bubble_radius: _______
- smooth_steering: ☐ true ☐ false
- adaptive_speed: ☐ true ☐ false

**Results:**
- Lap 1: _______ ☐ Completed ☐ Crashed
- Lap 2: _______ ☐ Completed ☐ Crashed
- Lap 3: _______ ☐ Completed ☐ Crashed
- Average: _______
- Best: _______

**Notes:**
_____________________________________________

---

### Test #12 - Best Combined Settings (Version 2)
**Date/Time**: _____________

**Description**: Refined version based on Test #11 observations

**Parameters:**
- max_speed: _______
- steering_gain: _______
- speed_gain: _______
- lookahead_distance: _______
- obstacle_bubble_radius: _______
- disparity_bubble_radius: _______
- smooth_steering: ☐ true ☐ false
- adaptive_speed: ☐ true ☐ false

**Results:**
- Lap 1: _______ ☐ Completed ☐ Crashed
- Lap 2: _______ ☐ Completed ☐ Crashed
- Lap 3: _______ ☐ Completed ☐ Crashed
- Average: _______
- Best: _______

**Notes:**
_____________________________________________

---

## CONSISTENCY TESTING

### Final Configuration Test - 10 Lap Endurance
**Date/Time**: _____________

**Configuration Name**: _____________

**Complete Parameter Set:**
```yaml
max_speed: _______
min_speed: _______
steering_gain: _______
speed_gain: _______
lookahead_distance: _______
field_of_vision: _______
obstacle_bubble_radius: _______
disparity_bubble_radius: _______
disparity_threshold: _______
smooth_steering: _______
steering_smoothing_alpha: _______
adaptive_speed: _______
gap_width_bonus: _______
```

**Results:**
| Lap # | Time | Status | Notes |
|-------|------|--------|-------|
| 1 | _____ | ☐ Complete ☐ Crash | _______ |
| 2 | _____ | ☐ Complete ☐ Crash | _______ |
| 3 | _____ | ☐ Complete ☐ Crash | _______ |
| 4 | _____ | ☐ Complete ☐ Crash | _______ |
| 5 | _____ | ☐ Complete ☐ Crash | _______ |
| 6 | _____ | ☐ Complete ☐ Crash | _______ |
| 7 | _____ | ☐ Complete ☐ Crash | _______ |
| 8 | _____ | ☐ Complete ☐ Crash | _______ |
| 9 | _____ | ☐ Complete ☐ Crash | _______ |
| 10 | _____ | ☐ Complete ☐ Crash | _______ |

**Statistics:**
- Completion Rate: _____% (___/10)
- Average Lap Time: _______
- Best Lap Time: _______
- Worst Lap Time: _______
- Standard Deviation: _______
- Consistency Score (lower is better): _______

---

## INSIGHTS & LEARNINGS

### What Worked Well
1. _____________________________________________
2. _____________________________________________
3. _____________________________________________

### What Didn't Work
1. _____________________________________________
2. _____________________________________________
3. _____________________________________________

### Track-Specific Observations
- **Straights**: _____________________________________________
- **Tight Corners**: _____________________________________________
- **Fast Corners**: _____________________________________________
- **Chicanes**: _____________________________________________
- **Problem Zones**: _____________________________________________

### Parameter Sensitivity Analysis
- **Most Important Parameter**: _____________
- **Least Important Parameter**: _____________
- **Most Surprising Finding**: _____________________________________________

---

## FINAL RACE CONFIGURATION

### Chosen Configuration: _____________

**Rationale:**
_____________________________________________
_____________________________________________

**Parameters:**
```yaml
# Copy your final config here
```

**Expected Performance:**
- Average Lap Time: _______
- Completion Probability: _______%
- Risk Level: ☐ Low ☐ Medium ☐ High

**Backup Configuration (if primary fails):**
_____________________________________________

---

## POST-RACE NOTES

### Actual Race Performance
- Qualifying Position: _______
- Final Position: _______
- Best Lap: _______
- Issues Encountered: _____________________________________________

### What Would You Change?
_____________________________________________
_____________________________________________

### Future Improvements
1. _____________________________________________
2. _____________________________________________
3. _____________________________________________
