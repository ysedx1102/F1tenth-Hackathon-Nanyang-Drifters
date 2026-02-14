# MECHANICAL ENGINEER #1 - UPDATED TASK LIST (4-PERSON TEAM)
## Role: Lead Tester & Steering Behavior Analyst

**Updated Time**: 25 hours (+3h from EE departure)

---

## ⚠️ WHAT CHANGED

You're now responsible for:
- ✅ **Steering behavior observation** (was EE's task)
- ✅ **Steering smoothing tuning** (collaborative with CS Student)
- ✅ **Speed profile analysis** (document actual vs target speeds)

**Why you?** You're already running all the tests - now you're watching steering more carefully!

**Good news**: Most of this integrates into your existing testing work.

---

## 📅 UPDATED TASK BREAKDOWN

### Week 1 Tasks (7 hours total) - MOSTLY UNCHANGED

#### Day 1-2: Track Analysis (3 hours)
- [ ] Load Nürburgring map
- [ ] Identify 8-10 key sections
- [ ] Sketch racing lines
- [ ] Note problem areas

**Same as before** - see original task list

---

#### Day 6-7: Baseline Testing (4 hours)
- [ ] Run 5 baseline laps
- [ ] Record lap times
- [ ] Note crash points
- [ ] Document behavior

**Same as before** - see original task list

---

### Week 2 Tasks (18 hours total) - SOME ADDITIONS

#### Day 8-9: Speed Testing + Steering Analysis (6 hours) - ADDED 2h

**Original: Speed Optimization (4h)**:
Test max_speed from 5.0 to 8.0 m/s (same as before)

**NEW: Steering Behavior Observation (2h)**:

While running speed tests, pay extra attention to steering:

**Observation Checklist**:
```markdown
### Steering Behavior Assessment

For EACH speed test, observe:

1. **Oscillation**:
   - [ ] Does the car wiggle/snake?
   - [ ] Frequency: ☐ None ☐ Slow ☐ Medium ☐ Fast
   - [ ] Severity: ☐ None ☐ Slight ☐ Moderate ☐ Severe
   
2. **Smoothness**:
   - [ ] Are steering transitions smooth?
   - [ ] Any sudden jerks?
   - [ ] Rating (1-10): _____

3. **Path Quality**:
   - [ ] Clean racing line?
   - [ ] Cuts corners? ☐ Yes ☐ No
   - [ ] Overshoots? ☐ Yes ☐ No

4. **Track Sections**:
   - Straights: ☐ Stable ☐ Wobbly
   - Gentle turns: ☐ Smooth ☐ Jerky
   - Sharp turns: ☐ Good ☐ Oversteers ☐ Understeers
```

**Record for CS Student**:
- Which speeds show most oscillation?
- Does wobbling get worse at higher speeds?
- Video record a lap (if possible) showing steering behavior

**Deliverable**: Speed test results + steering behavior notes

---

#### Day 9 Afternoon: Steering Smoothing Tuning Session (Collaborative, counted in Day 9 time)

**Work WITH CS Student**:

CS Student will prepare 5 configs with different smoothing:
- Alpha = 0.1 (very little smoothing)
- Alpha = 0.2
- Alpha = 0.3 (current default)
- Alpha = 0.4
- Alpha = 0.5 (heavy smoothing)

**Your Testing Protocol**:
For each alpha value:
1. Run 2 laps
2. Observe steering behavior carefully
3. Fill out observation form
4. Discuss with CS Student

**Observation Form** (fill one per alpha):
```markdown
## Alpha = _____

### Lap 1:
- Lap Time: _____
- Oscillations: ☐ None ☐ Slight ☐ Moderate ☐ Severe
- Steering feel: ☐ Too sensitive ☐ Good ☐ Too sluggish
- Path quality: ☐ Excellent ☐ Good ☐ Fair ☐ Poor

### Lap 2:
- Lap Time: _____
- Consistent with Lap 1? ☐ Yes ☐ No
- Any issues: _____________________

### Overall Rating (1-10): _____

### Notes:
_________________________________
```

**After all tests, discuss with CS Student**:
- Which alpha felt best?
- Trade-off between smoothness and responsiveness
- Choose together: Optimal alpha = _____

---

#### Day 10-11: Steering Testing + Speed Profiling (5 hours) - ADDED 1h

**Original: Steering Aggressiveness Testing (4h)**:

Test different steering_gain and speed_gain combinations (same as before)

**NEW: Speed Profile Documentation (1h)**:

During steering tests, specifically track actual speeds achieved:

**Speed Tracking Sheet**:
```markdown
## Speed Profile Analysis

Configuration: [Current parameters]

### Track Section Speeds:

| Section | Target Speed | Actual Speed | Gap | Notes |
|---------|--------------|--------------|-----|-------|
| Straight 1 | _____ | _____ | _____ | |
| Corner 1 Entry | _____ | _____ | _____ | |
| Corner 1 Apex | _____ | _____ | _____ | |
| Corner 1 Exit | _____ | _____ | _____ | |
| Straight 2 | _____ | _____ | _____ | |
| ... | | | | |

### Key Findings:
- Is car reaching max_speed on straights? ☐ Yes ☐ No
- If no, why? _____________________
- Is it slowing too much in gentle turns? ☐ Yes ☐ No
- Corner exit acceleration: ☐ Good ☐ Too cautious ☐ Too aggressive
```

**How to Measure**:
1. Watch the terminal output (speed is logged)
2. Or use RViz to visualize speed
3. Or ask CS Student to log speeds by section
4. Note approximate speeds in each section

**Share with CS Student**:
- Is speed control formula working as intended?
- Should max_speed be higher?
- Should speed_gain be adjusted?

---

#### Day 12-14: Same as Before
- Day 12: Test all 3 configurations (4h)
- Day 13: 10-lap endurance test (3h)
- Day 14: Final validation (1h)

**No changes** - see original task list

---

## 🤝 NEW COLLABORATION RESPONSIBILITIES

### With CS Student (New Partnership!)

**Day 9 Morning**: Speed tests
- You: Run tests and observe
- CS: Monitors remotely, available for bugs

**Day 9 Afternoon**: Smoothing tuning session (1.5-2 hours together)
- You: Drive tests and observe behavior
- CS: Adjust parameters
- Together: Decide on best value
- **This is a JOINT session - schedule it!**

**Day 11**: Speed profile feedback
- You: Send speed observations
- CS: Adjust speed control formula if needed

**Communication**:
- Schedule Day 9 afternoon session in advance
- Quick Slack updates after each test run
- Share observation forms immediately

---

### With Mech Eng #2 (Same as Before)

- You test → They analyze
- Share data continuously
- Compare findings daily

---

## 📊 ENHANCED OBSERVATION SKILLS

### What to Watch For (Priority Order)

**1. Oscillation (Most Critical)**:
- Slow wobble = smoothing too low
- No response = smoothing too high
- Sweet spot = smooth with slight corrections

**2. Path Quality**:
- Ideal: Smooth racing line, hits apex
- Too aggressive: Cuts corners, overshoots
- Too conservative: Wide turns, slow

**3. Speed Behavior**:
- Ideal: Fast on straights, controlled in corners
- Problem: Slow everywhere OR too fast and crashes

**4. Consistency**:
- Lap-to-lap variation
- Same behavior in similar corners?

---

## 🎥 DOCUMENTING BEHAVIOR

### Simple Video Recording (if possible)

**Option 1**: Screen record simulator
- Shows car behavior clearly
- Easy to review later
- Can share with team

**Option 2**: Phone video of screen
- Quick and easy
- Good enough for observations

**What to capture**:
- One full lap at each major config change
- Specific problem sections
- Before/after comparisons

**Use for**:
- Team discussions
- Your own analysis
- Final presentation

---

## ⚡ EMERGENCY SHORTCUTS (If Time Tight!)

### Minimum Testing (15 hours):

**Day 6-7**: Baseline (3h)
- Just 3 laps instead of 5
- Basic observations only

**Day 8-9**: Speed sweep (3h)
- Test only: 5.0, 6.0, 7.0, 8.0
- 2 laps each
- Basic steering notes

**Day 9**: Smoothing (1.5h)
- Test only 3 alphas: 0.2, 0.3, 0.4
- 1 lap each
- Quick decision

**Day 10-11**: Steering (3h)
- Test only 3 combinations
- Skip speed profiling detail

**Day 12**: Configs (3h)
- Test only top 2 configs
- 3 laps each

**Day 13**: Endurance (1.5h)
- 5 laps instead of 10

**Day 14**: Final (1h)
- Quick verification only

**Total: 15 hours** (vs 25 hours full plan)

---

## 🎯 YOUR UNIQUE VALUE

### Why Steering Analysis Matters

You're the only one who:
- ✅ Actually drives the car through all tests
- ✅ Sees the behavior live
- ✅ Can feel if something is "off"
- ✅ Knows the track intimately

**Your observations directly improve**:
- Steering smoothness
- Corner performance
- Overall lap times
- System reliability

**You're not just testing - you're tuning!**

---

## 📋 TESTING BEST PRACTICES (Reminders)

### Scientific Method
1. Change ONE thing at a time
2. Run multiple laps (consistency check)
3. Record EVERYTHING
4. Compare objectively

### Safety First
- If config crashes 2+ times, stop testing it
- Document crashes (where, when, why)
- Always have a safe backup config

### Communication
- Update team daily (async)
- Flag issues immediately
- Share wins (found a good config!)

---

## ✅ SUCCESS METRICS

### By Day 9:
- ✅ Optimal max_speed found
- ✅ Steering smoothness tuned with CS Student
- ✅ Clear understanding of car behavior

### By Day 11:
- ✅ Best steering configuration identified
- ✅ Speed profile documented
- ✅ Recommendations sent to CS Student

### By Day 14:
- ✅ Race config tested and verified
- ✅ 5+ consecutive successful laps
- ✅ Confident in system reliability

---

## 💪 YOU'VE GOT THIS!

**The extra 3 hours breaks down as**:
- 2h: Steering observation (while doing speed tests anyway)
- 1h: Speed profiling (while doing steering tests anyway)

**You're not doing MORE tests - just observing MORE CAREFULLY!**

**Think of it as**:
- Before: Test and record lap times
- Now: Test, record lap times, AND note steering behavior

**Your eye for vehicle dynamics is crucial!** 🏎️👁️

---

## 📞 When to Ask for Help

**Ask CS Student**:
- Need different smoothing values to test
- Speed control seems wrong
- Code behaving unexpectedly

**Ask Mech #2**:
- Need data analysis from tests
- Statistical comparison needed
- Which config is objectively better?

**Ask Comp Eng**:
- Simulator crashes
- Can't change parameters
- System won't start

---

Remember: You're the driver, the tester, AND now the steering analyst! Triple threat! 🏁
