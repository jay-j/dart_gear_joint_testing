# Objective
A unit test for using Mimic constraints to model the reflected inertia dynamics associated with electric gearmotors.

# Test
## Setup
A two link pendulum. Between the two links

## Expected Behavior
Increasing the gear ratio N will increase the apparent inertia of link2, thus for the same initial conditions (same initial energy input to the system) link2 will rotate less compared to link1. While in reality it is not uncommon for gearboxes to have ratios larger than 1000:1, for investigation of dynamic robots, stability for gear ratios up to 100:1 is highly desirable. 

## Observed Behavior
Test conditions:
- Linux Mint 
- DART 6.9.1


