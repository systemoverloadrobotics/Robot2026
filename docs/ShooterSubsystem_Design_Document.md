# ShooterSubsystem Design Proposal

**Team 6059 - FRC 2026**  
**Subsystem:** Shooter  
**Author:** [Your Name]  
**Date:** January 2026  
**Status:** Implemented - Ready for Hardware Testing  

---

## Purpose of This Document

I've been assigned to build the ShooterSubsystem for our 2026 robot. Before I start coding, I want to share my plan with the software lead to make sure I'm on the right track.

This document covers:
- What the shooter needs to do
- The hardware I'll be controlling
- How I'm planning to structure the code
- Questions I need answered before I start

**Please review and let me know if this approach looks good or if I should change anything.**

---

## Table of Contents

1. [What the Shooter Needs to Do](#1-what-the-shooter-needs-to-do)
2. [Hardware I'll Be Working With](#2-hardware-ill-be-working-with)
3. [How the Directions Will Work](#3-how-the-directions-will-work)
4. [My Planned Code Structure](#4-my-planned-code-structure)
5. [State Machine Design](#5-state-machine-design)
6. [Motor Control Approach](#6-motor-control-approach)
7. [Methods I Plan to Create](#7-methods-i-plan-to-create)
8. [Safety Features I'll Include](#8-safety-features-ill-include)
9. [How It Will Connect to Other Subsystems](#9-how-it-will-connect-to-other-subsystems)
10. [Tuning Plan](#10-tuning-plan)
11. [Testing Plan](#11-testing-plan)
12. [Questions I Need Answered](#12-questions-i-need-answered)

---

## 1. What the Shooter Needs to Do

Based on what I understand from the team, the shooter needs to:

1. **Control flywheel speed** - This determines how fast/far the ball goes
2. **Control hood angle** - This determines if we shoot left or right
3. **Report readiness** - Tell other subsystems when we're ready to shoot

### Key Design Rule

From the architecture doc I was given:

> "Shooter must NEVER feed balls on its own. It only reports readiness."

So my subsystem will just say "I'm ready" and let the StorageSubsystem handle actually feeding the ball. This keeps things clean and prevents conflicts.

### Two Modes I'll Support

| Mode | How it works | When we'd use it |
|------|--------------|------------------|
| **AUTO** | Vision gives distance → I look up RPM and angle | Normal match play |
| **MANUAL** | Driver sets RPM and angle directly | Testing, special shots |

---

## 2. Hardware I'll Be Working With

### Motors

Based on the mechanical design, we have 3 TalonFX motors:

| Motor | Purpose | My understanding |
|-------|---------|------------------|
| **Flywheel** | Launches the ball | Needs to spin fast (probably 2000-6000 RPM) |
| **Hood Angle** | Aims left or right | Position control to specific angles |
| **Hood Wheel** | Helps guide the ball | Spins opposite to flywheel |

### What I Need to Confirm

- [ ] CAN IDs for each motor
- [ ] Gear ratios (especially for hood angle)
- [ ] Any limit switches or sensors?
- [ ] Motor orientation (which way is "positive"?)

---

## 3. How the Directions Will Work

From what I understand, the hood can point in two directions:

```
        FRONT OF ROBOT
              ▲
              │
    ┌─────────┴─────────┐
    │                   │
    │   ◄── LEFT        │        RIGHT ──►
    │   (15°-45°)       │        (135°-165°)
    │                   │
    └───────────────────┘
```

### Direction Rules I'll Implement

| Direction | Hood Angle Range | Flywheel Spins | Hood Wheel Spins |
|-----------|------------------|----------------|------------------|
| **LEFT**  | 15° to 45° | Counter-clockwise | Clockwise |
| **RIGHT** | 135° to 165° | Clockwise | Counter-clockwise |

The flywheel and hood wheel spin opposite to each other - I think this creates a "steering" effect on the ball.

### How I'll Handle This in Code

I'm planning to have a `ShootDirection` enum (LEFT or RIGHT), and the code will automatically:
- Clamp the hood angle to the valid range for that direction
- Set the correct motor directions

So the user just calls `setShootDirection(LEFT)` and doesn't have to worry about the details.

---

## 4. My Planned Code Structure

I'm thinking of organizing it like this:

```
src/main/java/frc/robot/subsystems/shooter/
├── ShooterSubsystem.java      ← Main class with all the logic
├── ShooterConstants.java      ← All the tunable numbers
├── ShooterState.java          ← Enum for state machine states
├── ShootDirection.java        ← Enum for LEFT/RIGHT
├── ShotParameters.java        ← Class to hold RPM + angle together
└── ShooterLookupTable.java    ← Distance → shot parameters mapping
```

### Why Multiple Files?

I want to:
- Keep tunable values (CAN IDs, PID gains) in one place for easy changes
- Make the state machine states clear and type-safe
- Separate the lookup table logic so it's easy to update

**Is this a good structure? Should I organize it differently?**

---

## 5. State Machine Design

I'm planning to use a state machine to manage the shooter. This is what I'm thinking:

### The States

```
IDLE ──► SPINNING_UP ──► AIMING ──► READY ◄──► SHOOTING
                                       │
                                       ▼
                                    FAULT
```

| State | What it means | How we get here |
|-------|---------------|-----------------|
| `IDLE` | Everything off, waiting | Start state, or after disable |
| `SPINNING_UP` | Flywheel getting up to speed | After target is set |
| `AIMING` | Hood moving to target angle | After flywheel reaches speed |
| `READY` | Ready to shoot! | Both flywheel and hood at target |
| `SHOOTING` | Ball is being fed | Storage started feeding |
| `FAULT` | Something went wrong | Timeout or error detected |

### Transitions I'll Implement

1. **IDLE → SPINNING_UP**: When `setTargetDistance()` or `setTargetFlywheelRPM()` is called
2. **SPINNING_UP → AIMING**: When flywheel is within tolerance of target RPM
3. **AIMING → READY**: When hood is within tolerance of target angle
4. **READY → SHOOTING**: When `notifyShotStarted()` is called
5. **SHOOTING → READY**: After a short delay (shot complete) - **flywheel keeps spinning!**
6. **Any → FAULT**: If timeout or error occurs

### Continuous Shooting Design

**Key feature:** The flywheel **never stops** between shots!

```
READY → SHOOTING (0.25s) → READY → SHOOTING → READY → ...
        ↑                    ↑
        Flywheel keeps spinning the entire time
```

This allows rapid-fire shots without waiting for spinup between each shot.

### Hysteresis for Stability

To prevent bouncing between states, I use **different tolerances**:

| Check | Tolerance | Purpose |
|-------|-----------|---------|
| **Becoming READY** | ±75 RPM / ±1.5° | Must hit target precisely |
| **Staying READY** | ±150 RPM / ±3° (2x) | Small fluctuations won't interrupt |
| **After shot** | ±225 RPM (3x) | Even more tolerance since ball exit may cause brief dip |

This hysteresis prevents the state from bouncing between READY and SPINNING_UP due to normal RPM fluctuation.

### Timeout Values I'm Thinking

| Timeout | Value | Why |
|---------|-------|-----|
| Spinup timeout | 3 seconds | If flywheel can't reach speed, something's wrong |
| Aiming timeout | 2 seconds | Hood shouldn't take long to move |
| Shot duration | 0.25 seconds | Brief state while ball exits |

**Are these timeout values reasonable?**

---

## 6. Motor Control Approach

### Flywheel Control

I'm planning to use **velocity PID with feedforward**:

- **Feedforward**: Calculate roughly how much voltage we need for the target RPM
- **PID**: Fine-tune based on the error

I'll start with these values and tune from there:

| Gain | Starting Value | What it does |
|------|----------------|--------------|
| kV | 0.002 (12V / 6000 RPM) | Feedforward - volts per RPM |
| kS | 0.1 | Overcome static friction |
| kP | 0.1 | React to error |

### Hood Angle Control

For the hood, I'll use **position PID** since we care about where it is:

| Gain | Starting Value |
|------|----------------|
| kP | 0.05 |
| kD | 0.001 |

I'll also set soft limits in the TalonFX config to prevent going past mechanical limits.

### Lookup Table

For AUTO mode, I need a table that converts distance to RPM and angle:

```
Distance (m)  →  RPM    Angle Offset
1.0           →  2000   -5°
2.0           →  2600    0°
3.0           →  3200   +4°
4.0           →  3800   +8°
5.0           →  4400   +12°
6.0           →  5000   +14°
```

The "angle offset" will be added to a base angle:
- LEFT base: 30° (center of 15-45 range)
- RIGHT base: 150° (center of 135-165 range)

**These are placeholder values.** I'll need to tune them on the actual field.

For distances between entries, I'll use linear interpolation.

---

## 7. Methods I Plan to Create

### Core Control

```java
enableShooter()      // Turn on the subsystem
disableShooter()     // Turn off and stop motors
stopAllMotors()      // Emergency stop
```

### Direction and Targeting

```java
setShootDirection(ShootDirection direction)  // LEFT or RIGHT
setTargetDistance(double meters)             // AUTO mode - uses lookup table
setTargetFlywheelRPM(double rpm)             // MANUAL mode
setTargetHoodAngle(double degrees)           // MANUAL mode
```

### Convenience Methods

```java
prepareShot(ShootDirection direction, double distanceMeters)  // Set both at once
toggleShootDirection()  // Switch between LEFT and RIGHT
spinUp()               // Increase RPM by increment
spinDown()             // Decrease RPM by increment
```

### Status Methods

```java
boolean isAtSpeed()        // Is flywheel at target?
boolean isAtAngle()        // Is hood at target?
boolean isReadyToShoot()   // Both at target?
ShooterState getState()    // Current state
```

### The Key Method for Integration

```java
boolean isSafeToFeed()  // StorageSubsystem will check this
```

This will return `true` only when:
- Subsystem is enabled
- No faults
- State is READY
- Flywheel at speed
- Hood at angle

### Fault Handling

```java
boolean hasFault()
void clearFault()
void emergencyStop()
```

---

## 8. Safety Features I'll Include

### Angle Limiting

I'll make sure the hood angle is always clamped to the valid range for the current direction:

- LEFT mode: 15° to 45° only
- RIGHT mode: 135° to 165° only

If someone tries to set an invalid angle, I'll clamp it automatically.

### Soft Limits

I'll configure soft limits in the TalonFX to prevent the hood from ever going past 10° or 170° (mechanical limits).

### Fault Detection

I'm planning to detect these problems:

| Problem | How I'll detect it | What I'll do |
|---------|-------------------|--------------|
| Flywheel won't spin up | 3 second timeout | Go to FAULT |
| Hood won't move | 2 second timeout | Go to FAULT |
| Motor drawing too much current | Check motor current | Go to FAULT |
| Hood angle out of range | Check position | Go to FAULT |

### Emergency Stop

The `emergencyStop()` method will:
1. Immediately stop all motors
2. Set a fault flag
3. Go to FAULT state
4. Require `clearFault()` to recover

---

## 9. How It Will Connect to Other Subsystems

### With VisionSubsystem

Vision will tell us the distance to the target:

```java
// Vision calls:
shooter.setTargetDistance(vision.getDistance());
```

### With StorageSubsystem

This is the most important connection. Storage will check if it's safe to feed:

```java
// Storage logic:
if (shooter.isSafeToFeed()) {
    shooter.notifyShotStarted();
    // Feed the ball
}
```

The shooter never feeds on its own - it just says "I'm ready."

### With DriveSubsystem

I don't think I need to talk directly to Drive. Based on the architecture:
- Drive handles horizontal aiming (rotating the robot)
- Shooter handles vertical aiming (hood angle)

They both get info from Vision but work independently.

### With Superstructure

The Superstructure will coordinate everything:

```java
// Superstructure might do:
if (vision.hasTarget()) {
    drive.aimAt(vision.getYaw());
    shooter.setTargetDistance(vision.getDistance());
    
    if (drive.atTarget() && shooter.isSafeToFeed()) {
        shooter.notifyShotStarted();
        storage.feed();
    }
}
```

---

## 10. Tuning Plan

### Phase 1: Basic Motor Testing

1. Verify CAN IDs and wiring
2. Test each motor individually
3. Make sure directions are correct

### Phase 2: PID Tuning

**Flywheel:**
1. Start with just feedforward (kV)
2. Run at 50% target RPM
3. Adjust kV until steady-state is close
4. Add kP to fix remaining error
5. Add kD if there's overshoot

**Hood:**
1. Start with low kP
2. Command small movements (5-10°)
3. Increase kP until responsive
4. Add kD to reduce oscillation

### Phase 3: Lookup Table Tuning

1. Set up targets at known distances (1m, 2m, 3m, etc.)
2. For each distance:
   - Start with estimated values
   - Fire shots and observe
   - Adjust RPM until distance is right
   - Adjust angle until accuracy is right
   - Record final values
3. Update lookup table with real values

---

## 11. Testing Plan

### Simulation Testing (WPILib Simulator)

I've built a test mode that can run in the WPILib simulator:

- **TestShooterRobotContainer.java** - Separate RobotContainer for testing
- **Automatic test sequence** - Runs D-Pad/angle tests automatically
- **Controller support** - Full Xbox controller bindings for manual testing

**Simulation notes:**
- D-Pad (POV) doesn't work in simulation - use joysticks instead
- RPM/angle checks skip in simulation (motors don't have physics)
- Fault detection disabled in simulation

See **ShooterSubsystem_Testing_Guide.md** for detailed instructions.

### Unit Tests (Before Hardware)

- [ ] State machine transitions work correctly
- [ ] Angle clamping works for both directions
- [ ] Lookup table interpolation is correct
- [ ] Motor direction logic is correct

### Hardware Tests

- [ ] Each motor spins in the expected direction
- [ ] Flywheel reaches and holds target RPM
- [ ] Hood moves to target angle accurately
- [ ] Direction toggle reverses motors correctly
- [ ] Emergency stop works immediately
- [ ] Continuous shooting works (multiple shots without stopping)

### Integration Tests

- [ ] Vision can set target distance
- [ ] `isSafeToFeed()` returns true when ready
- [ ] Full shot sequence works
- [ ] Multiple shots in sequence work (continuous shooting)
- [ ] Hysteresis prevents state bouncing

---

## 12. Questions I Need Answered

Before I start coding, I need help with these:

### Hardware Questions

1. **What are the CAN IDs?** I need the actual IDs for flywheel, hood angle, and hood wheel motors.

2. **What are the gear ratios?** Especially for the hood angle mechanism - I need this to convert motor rotations to degrees.

3. **How should I zero the hood encoder?** Is there a limit switch? A known starting position? Should I add a calibration routine?

4. **Which way is "positive" for each motor?** I want to make sure my inversion settings are correct.

### Software Questions

5. **Is my state machine design okay?** Should I add or remove any states?

6. **Are my timeout values reasonable?** 3 seconds for spinup, 2 seconds for aiming - too long? Too short?

7. **Should the lookup table be in code or a file?** I could load it from a JSON file if that's easier to tune.

8. **What tolerances should I use?** I implemented ±75 RPM for flywheel and ±1.5° for hood to *become* READY, with 2x tolerance (±150 RPM / ±3°) to *stay* READY (hysteresis). Is this a good approach?

### Integration Questions

9. **How does VisionSubsystem work?** What method do I call to get the distance?

10. **How will Superstructure coordinate everything?** Should I wait for that to be designed, or make assumptions?

11. **Who's building StorageSubsystem?** I need to coordinate with them on the `isSafeToFeed()` interface.

---

## Summary

Here's what I'm planning to build:

- A ShooterSubsystem with 3 TalonFX motors (flywheel, hood angle, hood wheel)
- State machine with 6 states (IDLE, SPINNING_UP, AIMING, READY, SHOOTING, FAULT)
- Support for LEFT and RIGHT shooting directions
- AUTO mode (distance-based) and MANUAL mode
- Lookup table for distance → RPM/angle conversion
- Safety features including angle limits, timeouts, and fault detection
- Key integration method: `isSafeToFeed()` for StorageSubsystem

### Key Implementation Features

| Feature | Description |
|---------|-------------|
| **Continuous Shooting** | Flywheel never stops between shots - ready for rapid fire |
| **Hysteresis** | Different tolerances for entering vs. staying in READY state |
| **Simulation Support** | TestShooterRobotContainer for testing without hardware |
| **Controller Mapping** | Full Xbox controller support with joystick alternatives |

### Files Created

```
src/main/java/frc/robot/subsystems/shooter/
├── ShooterSubsystem.java      ← Main subsystem (905 lines)
├── ShooterConstants.java      ← All configurable values
├── ShooterState.java          ← State machine enum
├── ShootDirection.java        ← LEFT/RIGHT enum
├── ShotParameters.java        ← RPM + angle data class
└── ShooterLookupTable.java    ← Distance → shot parameters

src/main/java/frc/robot/
└── TestShooterRobotContainer.java  ← Test mode controller

docs/
├── ShooterSubsystem_Design_Document.md   ← This document
└── ShooterSubsystem_Testing_Guide.md     ← How to test
```

**Please let me know:**
- If this approach looks good
- What I should change
- Answers to my questions above

Thanks for reviewing!

---

*[Your Name]*  
*Team 6059*
