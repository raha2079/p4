# Improvements Made to Checkpoint Submission

## Issues Addressed

### 1. ✅ KPIECE Not in Report
**Problem:** Original report only showed RRT results
**Solution:**
- Implemented projections for both systems
- Ran comprehensive tests with both RRT and KPIECE
- Created side-by-side comparison visualizations
- Added detailed RRT vs KPIECE analysis to report

### 2. ✅ Car Goal Too Close to Obstacle
**Problem:** Goal position (3, 5.5) was inside right obstacle block
**Solution:**
- Fixed goal position to (-2, 1) - safely in free corridor
- Verified start (-2, -5) and goal are both collision-free
- Tested with all configurations - all succeed

### 3. ✅ Explain WHY Pendulum Paths Differ
**Problem:** No physics-based explanation for torque effects
**Solution:** Added detailed explanations:

**τ=3 (Low Torque):**
- Physics: τ_max < g·cos(θ) for most angles → cannot directly fight gravity
- Strategy: Momentum-building through oscillations (like pumping a swing)
- Result: Multiple loops in phase space, energy accumulation

**τ=5 (Medium Torque):**
- Physics: Partial control authority
- Strategy: Hybrid - some direct control, some momentum building
- Result: 1-2 large swings with active damping

**τ=10 (High Torque):**
- Physics: τ_max > g → can overcome gravity everywhere
- Strategy: Quasi-static motion possible
- Result: Direct, smooth paths with minimal oscillation

### 4. ✅ Test Multiple Car Configurations
**Problem:** Only tested 1 car configuration
**Solution:** Tested 5 configurations:

| Config | Controls | Difficulty | Results |
|--------|----------|------------|---------|
| 1 | ω=±0.5, a=±1 | Very Hard | ✓ Both planners succeed |
| 3 | ω=±1, a=±2 | Med-Hard | ✓ More maneuverable |
| 5 | ω=±2, a=±3 | Medium | ✓ Balanced (recommended) |
| 7 | ω=±3, a=±4 | Easy | ✓ High authority |
| 9 | ω=±4, a=±5 | Very Easy | ✓ Very direct paths |

**Key Findings:**
- Tighter limits → longer paths, cautious navigation
- Looser limits → shorter paths, aggressive maneuvers
- Non-holonomic constraints persist regardless of control authority

### 5. ✅ More Pendulum Tests
**Problem:** Only had 3 pendulum tests
**Solution:**
- Tested each torque (3, 5, 10) with BOTH planners (RRT and KPIECE)
- Total: 6 pendulum tests (3 torques × 2 planners)
- Analyzed differences between RRT and KPIECE solutions
- Created dedicated comparison visualizations

---

## New Visualizations (3 Figures)

### Figure 1: `pendulum_rrt_vs_kpiece.png`
- Side-by-side comparison of RRT vs KPIECE
- Shows all 3 torque values for each planner
- Highlights differences in exploration strategies

### Figure 2: `pendulum_torque_analysis.png`
- Individual detailed view of each torque level
- Both RRT and KPIECE overlaid for each
- Emphasizes physical behavior differences

### Figure 3: `car_multiple_configs.png`
- 5 different control configurations shown
- RRT (solid blue) vs KPIECE (dashed red) for each
- Demonstrates effect of control limits on planning

---

## Comprehensive Test Results

**Total Tests Run:** 16
- Pendulum: 3 torques × 2 planners = 6 tests
- Car: 5 configs × 2 planners = 10 tests

**Success Rate:** 100% (16/16 passed)

**Files Generated:**
```
Pendulum RRT:     pend_rrt_t3.txt, pend_rrt_t5.txt, pend_rrt_t10.txt
Pendulum KPIECE:  pend_kpiece_t3.txt, pend_kpiece_t5.txt, pend_kpiece_t10.txt
Car RRT:          car_rrt_c1.txt, car_rrt_c3.txt, car_rrt_c5.txt,
                  car_rrt_c7.txt, car_rrt_c9.txt
Car KPIECE:       car_kpiece_c1.txt, car_kpiece_c3.txt, car_kpiece_c5.txt,
                  car_kpiece_c7.txt, car_kpiece_c9.txt
```

---

## New Report Structure

**Checkpoint_Report_FINAL.md includes:**

1. **Pendulum System**
   - Problem description with full dynamics
   - RRT vs KPIECE comparison (with figure)
   - Detailed torque analysis (with figure)
   - Physics-based explanations for each torque level
   - RRT vs KPIECE behavioral differences

2. **Car System**
   - Problem description with kinematics
   - Table of all 5 configurations tested
   - Results visualization (with figure)
   - Configuration-by-configuration analysis
   - Effect of control limits on planning difficulty

3. **Implementation Summary**
   - Projection implementations explained
   - State validity checking details
   - ODE integration approach
   - Comprehensive test matrix (16 tests)

4. **Conclusions**
   - 5 key takeaways from experiments
   - Trade-offs between planners
   - Effect of control authority

---

## What Changed in Code

1. **Car goal position:** Fixed to avoid obstacles
2. **Projections:** Fully implemented for KPIECE
3. **No other code changes** - all implementations were already correct

---

## Files to Submit

**Main Report (convert to PDF):**
- `Checkpoint_Report_FINAL.md` ← Use this one!

**Figures (embed in PDF):**
- `pendulum_rrt_vs_kpiece.png` (697 KB)
- `pendulum_torque_analysis.png` (465 KB)
- `car_multiple_configs.png` (386 KB)

**Source Code:**
- All `.cpp` and `.h` files in `src/`
- Includes full RRT and KPIECE implementations
- Includes projections for KPIECE
- All 16 test configurations work

---

## Submission Checklist

- [ ] Convert `Checkpoint_Report_FINAL.md` to PDF
- [ ] Embed all 3 PNG figures in the PDF
- [ ] Run `prepare_submission.sh` to create ZIP
- [ ] Upload ZIP + PDF to Canvas
- [ ] **Deadline: TODAY at 1pm!**

