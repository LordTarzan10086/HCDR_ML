# Microgravity Static Feasibility Fix - Implementation Notes

## Problem Summary

The original code showed all configurations as statically infeasible in microgravity scenarios due to:

1. **Incorrect treatment of perturbation as external load**: The LP treated `fz_eps=2N` as a required external load that must be balanced
2. **Hard tension lower bound conflict**: `tau_min=5N` created a minimum combined force that made small external loads infeasible
3. **Missing self-stress check**: No verification that the configuration could maintain tension under zero external load

## Solution Implemented

### 1. New Self-Stress Check Function (`check_self_stress`)

Added to `HCDR_statics_5d.m` as the primary gate for microgravity feasibility:

```matlab
function [rho0_max, T0, info] = check_self_stress(A5, config, options)
```

**Key features:**
- Solves: `max rho s.t. A5*T = 0, T >= tau_min + rho, T <= tau_max`
- Returns positive `rho0_max` if configuration can hover at zero load
- Uses relaxed `tau_min_solve = 0.5N` instead of `tau_min = 5N` for search
- This is the **correct mathematical condition** for microgravity hovering

### 2. Configuration Changes (`HCDR_config_v2.m`)

**Critical changes:**
```matlab
% OLD (incorrect):
config.microg.W5_nominal = [0; 0; -config.microg.fz_eps; 0; 0];

% NEW (correct):
config.microg.W5_nominal = zeros(5,1);  % Zero nominal load in microgravity!
config.microg.fz_eps = 2.0;  % Only for disturbance testing
```

**New parameters:**
```matlab
config.cable.tau_min_phys = 5.0;   % Physical desired pretension
config.cable.tau_min_solve = 0.5;  % Relaxed for feasibility search
```

### 3. Modified Tension Solver (`solve_tension_optimal`)

**New workflow:**
1. **First**: Check self-stress with relaxed bounds → gate condition
2. **Second**: Check feasibility with external wrench (if any)
3. **Third**: Solve QP for optimal tension distribution

```matlab
% STEP 1: Self-stress check (NEW!)
[rho0, T0_ss, info_ss] = HCDR_statics_5d.check_self_stress(A5, config);
if rho0 <= 0
    warning('No self-stress available!');
    return;  % Fail early
end

% STEP 2: External wrench check (existing)
[is_feasible, rho_max, T_init] = check_feasibility(A5, W5, config);
```

### 4. Diagnostic Helper Function

Added `print_diagnostics` to help debug static feasibility issues:

**Outputs:**
- Cable z-components (uz) - shows if all cables pull same direction
- Achievable Fz range [Fz_min, Fz_max] based on tau bounds
- Self-stress margin (rho0)
- External wrench feasibility

**Usage:**
```matlab
HCDR_statics_5d.print_diagnostics(A5, W5, config, U);
```

## Why This Fixes The Problem

### Before (Incorrect Physics):
- System tried to balance `fz_eps=2N` with `tau_min=5N` on all cables
- If all cables point upward: minimum achievable Fz = 8 × 5N × uz_avg >> 2N
- Result: LP always infeasible, even though configuration is physically sound

### After (Correct Physics):
- System first checks: "Can we maintain tension with zero external load?" (self-stress)
- If yes (rho0 > 0): Configuration can hover in microgravity ✓
- Then optionally check: "Can we handle small disturbances?" (robustness)
- `fz_eps` is now correctly used only for robustness testing, not as required load

## Testing

### Test Script: `test_self_stress.m`

Validates the implementation with three configurations:
1. **Initial configuration** (centered, no tilt)
2. **Offset configuration** (displaced + small roll/pitch)
3. **Edge configuration** (near workspace boundary)

**Expected results:**
- All should show `rho0 > 0` (self-stress exists)
- Diagnostics show uz components and achievable force ranges
- Zero nominal load (W5_nominal) should be feasible

### Integration with Planning

`plan_platform_only.m` now:
- Calls `solve_tension_optimal` which gates on self-stress first
- Prints diagnostics when infeasibility detected
- Optimization can find feasible solutions by adjusting platform pose and slider heights

## Mathematical Background

**Self-stress** in cable-driven systems is the ability to maintain internal tension without external loads:

```
A5 * T = 0
T >= tau_min
```

This is essential in microgravity because:
- No gravity to "pull against"
- Must create internal force balance through geometry
- Enables stiffness/controllability without external support

**Key insight:** In microgravity, `tau_min > 0` makes ZERO external load HARDER than non-zero loads, which is counter-intuitive but mathematically correct.

## Backward Compatibility

- `check_feasibility` retained with updated documentation
- `config.cable.tau_min` still exists for backward compatibility
- Existing test scripts should work (may see different warnings)
- No breaking changes to public APIs

## Files Modified

1. `HCDR_statics_5d.m` - Added self-stress check, modified solver, added diagnostics
2. `HCDR_config_v2.m` - Changed W5_nominal to zero, added tau_min_solve
3. `plan_platform_only.m` - Integrated diagnostics on infeasibility
4. `test_self_stress.m` - New comprehensive test script

## References

See `HCDR_v2.3.md` sections 1-6 for complete theoretical analysis and derivation.
