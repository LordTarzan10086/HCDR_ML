#!/usr/bin/env python3
"""
Verification script for microgravity static feasibility fix
"""

import re

def check_file_contains(filename, patterns, description):
    """Check if file contains all specified patterns"""
    print(f"\nChecking {filename} - {description}:")
    try:
        with open(filename, 'r') as f:
            content = f.read()
        
        for pattern in patterns:
            if re.search(pattern, content, re.MULTILINE | re.DOTALL):
                print(f"  ✓ Found: {pattern[:60]}...")
            else:
                print(f"  ✗ Missing: {pattern[:60]}...")
                return False
        return True
    except Exception as e:
        print(f"  ✗ Error: {e}")
        return False

# Verification checklist
checks = [
    ("HCDR_statics_5d.m", [
        r"function \[rho0_max, T0, info\] = check_self_stress",
        r"A5 \* T = 0.*zero external load",
        r"tau_min_solve",
        r"function.*print_diagnostics",
        r"Self-stress margin.*rho0",
        r"Achievable Fz range",
    ], "Self-stress function and diagnostics"),
    
    ("HCDR_config_v2.m", [
        r"W5_nominal = zeros\(5,1\)",
        r"tau_min_phys = 5\.0",
        r"tau_min_solve = 0\.5",
        r"fz_eps.*disturbance testing",
    ], "Config changes for zero nominal load"),
    
    ("plan_platform_only.m", [
        r"solve_tension_optimal",
        r"print_diagnostics",
    ], "Platform planner integration"),
    
    ("plan_arm_only.m", [
        r"check_self_stress",
        r"has_self_stress",
        r"self_stress_margin",
    ], "Arm planner integration"),
    
    ("plan_cooperative.m", [
        r"check_self_stress",
        r"has_self_stress",
        r"rho0",
    ], "Cooperative planner integration"),
    
    ("test_self_stress.m", [
        r"check_self_stress",
        r"print_diagnostics",
        r"Self-stress margin",
    ], "Test script"),
    
    ("IMPLEMENTATION_NOTES.md", [
        r"check_self_stress",
        r"W5_nominal = zeros",
        r"Mathematical Background",
    ], "Documentation"),
]

print("=" * 70)
print("VERIFICATION REPORT: Microgravity Static Feasibility Fix")
print("=" * 70)

all_passed = True
for filename, patterns, description in checks:
    passed = check_file_contains(filename, patterns, description)
    if not passed:
        all_passed = False

print("\n" + "=" * 70)
if all_passed:
    print("✓ ALL CHECKS PASSED")
    print("\nImplementation appears complete. Key changes:")
    print("  1. Added check_self_stress() for zero-load feasibility")
    print("  2. Changed W5_nominal to zeros (microgravity!)")
    print("  3. Added tau_min_solve for relaxed search")
    print("  4. Integrated self-stress checks in all planners")
    print("  5. Added comprehensive diagnostics")
else:
    print("✗ SOME CHECKS FAILED")
    print("\nPlease review the missing items above")

print("=" * 70)
