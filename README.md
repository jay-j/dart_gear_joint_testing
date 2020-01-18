# Objective
A unit test for using Mimic constraints to model the reflected inertia dynamics associated with electric gearmotors.

Relevant DartSim github issues
- [Support Mimic Joint #733](https://github.com/dartsim/dart/issues/733)
- [Consider supporting transmission constraints #1047](https://github.com/dartsim/dart/issues/1047)
- [Add support for mimic joints #1178](https://github.com/dartsim/dart/pull/1178)

# Test
## Setup
A two link pendulum. Between the two links the revolue joint is modeled as a gearmotor; there is an additional body acting as the `rotor` for a hypothetical motor (allowed to spin freely in this simulation) on the joint between `link1` and `link2`. The `link1` is the parent of `rotor`, but `rotor` is constrained to rotate `ratio`X the rotations of the `link1`-`link2` joint. 

## Expected Behavior
Increasing the gear ratio `N` will increase the apparent inertia of `link2`, thus for the same initial conditions (same initial energy input to the system) `link2` will rotate less compared to `link1`. While in reality it is not uncommon for gearboxes to have ratios larger than 1000:1, for investigation of dynamic robots, stability for gear ratios up to 100:1 is highly desirable. 

## Observed Behavior
Test conditions:
- Linux Mint 
- DART 6.9.1

Compiles without error or warning. 

With `ratio=2` the simulation runs and by eye does appear to have the intended qualitative effect.

But with `ratio=4` the simulation goes unstable and gives an error:
Error [BoxedLcpConstraintSolver.cpp:291] [BoxedLcpConstraintSolver] The solution of LCP includes NAN values: nan. We're setting it zero for safety. Consider using more robust solver such as PGS as a secondary solver. If this happens even with PGS solver, please report this as a bug.

This error persists even after attempting to enable the PGS solver. Maybe I'm doing it incorrectly?
194   auto lcpSolver = std::make_shared<dart::constraint::PgsBoxedLcpSolver>();
195   auto solver = std::make_unique<dart::constraint::BoxedLcpConstraintSolver>(lcpSolver);
196   world->setConstraintSolver(std::move(solver));

