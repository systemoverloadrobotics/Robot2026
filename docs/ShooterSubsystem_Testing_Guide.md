# ShooterSubsystem Testing Guide

**Team 6059 - FRC 2026**  
**Date:** January 2026  

---

## Overview

This guide explains how to test the ShooterSubsystem in simulation and on hardware.

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Controller Button Mapping](#2-controller-button-mapping)
3. [Simulation Testing](#3-simulation-testing)
4. [Hardware Testing](#4-hardware-testing)
5. [Test Scenarios](#5-test-scenarios)
6. [Dashboard Values to Monitor](#6-dashboard-values-to-monitor)
7. [Troubleshooting](#7-troubleshooting)
8. [Test Checklist](#8-test-checklist)

---

## 1. Prerequisites

### Software Setup

1. **Enable Test Mode in Robot.java**
   
   Open `src/main/java/frc/robot/Robot.java` and change line 19 and 28:

   ```java
   // Line 19: Change type
   private final TestShooterRobotContainer m_robotContainer;
   
   // Line 28: Change constructor
   m_robotContainer = new TestShooterRobotContainer();
   ```

2. **Build the project**
   ```bash
   ./gradlew build
   ```

### Hardware Requirements (for hardware testing)

- Xbox controller (or compatible gamepad)
- Robot with ShooterSubsystem motors connected
- Driver Station laptop

---

## 2. Controller Button Mapping

```
┌────────────────────────────────────────────────────────────────┐
│                     XBOX CONTROLLER LAYOUT                      │
├────────────────────────────────────────────────────────────────┤
│                                                                 │
│        [LB] Spin Down              [RB] Spin Up                │
│          -250 RPM                   +250 RPM                    │
│                                                                 │
│        [LT] Preset:                [RT] Preset:                │
│         LEFT @ 2m                   RIGHT @ 4m                  │
│                                                                 │
│                                                                 │
│       LEFT STICK                   RIGHT STICK                  │
│      ← Direction →                ↑ Hood +5°                   │
│      (LEFT / RIGHT)               ↓ Hood -5°                   │
│                                                                 │
│                                                                 │
│                        [Y] Toggle Direction                     │
│                                                                 │
│                    [X]                [B]                       │
│               Prepare Shot         Fire Shot                    │
│               (RIGHT, 3m)         (if ready)                    │
│                                                                 │
│                        [A] Enable/Disable                       │
│                                                                 │
│                                                                 │
│       [BACK]                              [START]               │
│     Clear Fault                       EMERGENCY STOP            │
│                                                                 │
│                                                                 │
│   NOTE: D-Pad may not work in simulation.                       │
│   Use Left/Right Stick as shown above instead!                  │
│                                                                 │
└────────────────────────────────────────────────────────────────┘
```

### Quick Reference Table

| Control | Action | Expected Result |
|---------|--------|-----------------|
| **A Button** | Enable/Disable | Toggle shooter on/off |
| **B Button** | Fire | Triggers shot if ready |
| **X Button** | Prepare shot | Sets RIGHT direction, 3m distance |
| **Y Button** | Toggle direction | Switches between LEFT and RIGHT |
| **RB** | Spin up | Increases RPM by 250 |
| **LB** | Spin down | Decreases RPM by 250 |
| **Right Stick UP** | Angle +5° | Increases hood angle |
| **Right Stick DOWN** | Angle -5° | Decreases hood angle |
| **Left Stick LEFT** | Set LEFT | Changes to LEFT direction |
| **Left Stick RIGHT** | Set RIGHT | Changes to RIGHT direction |
| **Left Trigger** | LEFT 2m preset | Prepares LEFT shot at 2 meters |
| **Right Trigger** | RIGHT 4m preset | Prepares RIGHT shot at 4 meters |
| **Start** | Emergency stop | Immediately stops all motors |
| **Back** | Clear fault | Clears fault and returns to IDLE |

### D-Pad (Hardware Only)

The D-Pad may not work in WPILib simulation. On real hardware:

| D-Pad | Action |
|-------|--------|
| **D-Pad Up** | Hood angle +5° |
| **D-Pad Down** | Hood angle -5° |
| **D-Pad Left** | Set LEFT direction |
| **D-Pad Right** | Set RIGHT direction |

---

## 3. Simulation Testing

### Step 1: Start the Simulation

```bash
cd /Users/nbedarkar/Documents/NB/Ayush/Robot2026
./gradlew simulateJava
```

### Step 2: Configure the Simulation GUI

1. **Find the "Robot State" panel** (left side of simulation window)

2. **Enable Teleoperated Mode:**
   - Click **"Teleoperated"**
   - The robot will automatically enable

3. **Automatic Test Runs!**
   
   When you enable Teleoperated mode, an **automatic test sequence** runs:
   ```
   ========================================
     TELEOP STARTED - D-PAD TEST RUNNING
     Testing hood angle controls!
   ========================================
   ```
   
   This tests all D-Pad functions automatically without needing a controller!

### Step 3: Connect a Controller (Optional)

**With USB Controller:**
1. Plug in Xbox controller **before** starting simulation
2. In "System Joysticks" panel, verify "0: Controller" appears
3. Joystick warnings are silenced automatically

**Important Simulation Notes:**
- D-Pad (POV) often doesn't work in simulation
- Use **Right Stick** for hood angle (UP/DOWN)
- Use **Left Stick** for direction (LEFT/RIGHT)
- All buttons (A, B, X, Y, LB, RB, triggers) work normally

### Step 4: Monitor Values

**Option A: SmartDashboard/Shuffleboard**
```bash
cd ~/wpilib/2025/tools && java -jar ShuffleBoard.jar
```

**Option B: Glass**
```bash
open ~/wpilib/2025/tools/Glass.app
```

Look for these values:

```
Shooter/State           → Current state machine state
Shooter/Direction       → LEFT or RIGHT
Shooter/Enabled         → true/false
Shooter/TargetRPM       → Target flywheel RPM
Shooter/CurrentRPM      → Actual RPM (simulated)
Shooter/TargetAngle     → Target angle in degrees
Shooter/CurrentAngle    → Actual angle (simulated)
Shooter/ReadyToShoot    → true when ready to fire
Shooter/SafeToFeed      → true when storage can feed
Controller/POV          → D-Pad value (-1 if not pressed)
Controller/RightX       → Right stick X axis
Controller/RightY       → Right stick Y axis
```

### Step 5: Manual Test Sequence (with controller)

1. Press **A** → Shooter enables (State: IDLE)
2. Press **X** → Prepare shot (State: SPINNING_UP → AIMING → READY)
3. Move **Right Stick UP/DOWN** → Adjust hood angle ±5°
4. Press **B** → Fire shot (State: SHOOTING → READY)
5. Move **Left Stick LEFT** → Switch to LEFT direction
6. Move **Left Stick RIGHT** → Switch to RIGHT direction
7. Press **Y** → Toggle direction (alternative)
8. Press **RB/LB** → Adjust RPM ±250

### Console Output

Watch the terminal for real-time feedback:
```
[CONTROLLER] Button detected!
  → A pressed
[TEST] Shooter ENABLED
[RIGHT STICK UP] Hood angle +5°: 150.0° → 155.0°
[STICK LEFT] Direction: LEFT, Hood angle: 45.0°
```

---

## 4. Hardware Testing

### Step 1: Deploy to Robot

```bash
./gradlew deploy
```

### Step 2: Connect Driver Station

1. Connect laptop to robot (USB, Ethernet, or WiFi)
2. Open FRC Driver Station
3. Wait for connection

### Step 3: Open Dashboard

**Option A: Shuffleboard**
```bash
cd ~/wpilib/2025/tools && java -jar ShuffleBoard.jar
```

**Option B: Glass**
```bash
open ~/wpilib/2025/tools/Glass.app
```

### Step 4: Safety First!

Before enabling:

- [ ] Clear area around robot
- [ ] Shooter has no obstructions
- [ ] Everyone is aware robot is about to move
- [ ] Safety glasses on
- [ ] Know where the emergency stop is

### Step 5: Test Sequence

1. **Enable in Teleop mode**

2. **Press A to enable shooter**
   - Listen for any unusual sounds
   - Check dashboard shows State: IDLE

3. **Press X to prepare shot**
   - Watch flywheel spin up
   - Watch hood move to angle
   - Check dashboard for state transitions

4. **Verify targets are reached:**
   - `Shooter/Flywheel/CurrentRPM` ≈ `TargetRPM`
   - `Shooter/Hood/CurrentAngle` ≈ `TargetAngle`
   - `Shooter/ReadyToShoot` = true

5. **Test direction toggle (Y)**
   - Motors should reverse direction
   - Hood should move to opposite range

6. **Test emergency stop (Start)**
   - All motors should stop immediately
   - State should go to FAULT

---

## 5. Test Scenarios

### Scenario 1: Basic Enable/Disable

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1 | Press A | State: IDLE, Enabled: true |
| 2 | Press A again | State: IDLE, Enabled: false, motors stop |

### Scenario 2: Auto Shot Preparation

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1 | Press A | Shooter enabled |
| 2 | Press X | State: SPINNING_UP |
| 3 | Wait | State: AIMING (when RPM reached) |
| 4 | Wait | State: READY (when angle reached) |
| 5 | Check | ReadyToShoot = true |

### Scenario 3: Direction Toggle

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1 | Enable + Prepare (A, X) | Direction: RIGHT, Angle: ~150° |
| 2 | Press Y | Direction: LEFT, Angle adjusts to ~30° |
| 3 | Press Y | Direction: RIGHT, Angle adjusts to ~150° |

### Scenario 4: Manual RPM Control

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1 | Enable (A) | Shooter enabled |
| 2 | Press RB x4 | TargetRPM increases by 1000 |
| 3 | Press LB x2 | TargetRPM decreases by 500 |

### Scenario 5: Emergency Stop Recovery

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1 | Enable + Prepare | State: READY |
| 2 | Press Start | State: FAULT, all motors stop |
| 3 | Press Back | Fault cleared, State: IDLE |
| 4 | Press A | Can enable again |

### Scenario 6: Full Shot Sequence

| Step | Action | Expected Result |
|------|--------|-----------------|
| 1 | Press A | Enable shooter |
| 2 | Press X | Prepare shot |
| 3 | Wait for READY | ReadyToShoot = true |
| 4 | Press B | State briefly goes to SHOOTING |
| 5 | Wait | State returns to READY |

---

## 6. Dashboard Values to Monitor

### Critical Values

| Key | Good Value | Problem If |
|-----|------------|------------|
| `Shooter/State` | Transitions correctly | Stuck in one state |
| `Shooter/ReadyToShoot` | true when at target | Never becomes true |
| `Shooter/HasFault` | false | true (check FaultMessage) |

### Flywheel Values

| Key | What to Check |
|-----|---------------|
| `Shooter/Flywheel/TargetRPM` | Changes when you set target |
| `Shooter/Flywheel/CurrentRPM` | Approaches TargetRPM |
| `Shooter/Flywheel/AtSpeed` | true when within tolerance |

### Hood Values

| Key | What to Check |
|-----|---------------|
| `Shooter/Hood/TargetAngle` | Within valid range for direction |
| `Shooter/Hood/CurrentAngle` | Approaches TargetAngle |
| `Shooter/Hood/AtAngle` | true when within tolerance |

### Direction-Specific Checks

| Direction | Valid Angle Range | Base Angle |
|-----------|-------------------|------------|
| LEFT | 15° - 45° | 30° |
| RIGHT | 135° - 165° | 150° |

---

## 7. Troubleshooting

### Problem: Shooter won't enable

**Possible causes:**
- Fault is present → Press Back to clear
- Check `Shooter/HasFault` and `Shooter/FaultMessage`

### Problem: Stuck in SPINNING_UP

**Possible causes:**
- PID not tuned correctly
- Motor not connected
- Wrong CAN ID

**Solutions:**
- Check motor is spinning
- Verify CAN ID in ShooterConstants.java
- Tune flywheel PID gains

### Problem: Stuck in AIMING

**Possible causes:**
- Hood motor not responding
- Angle out of valid range
- PID not tuned

**Solutions:**
- Check hood motor connection
- Verify angle is within direction limits
- Tune hood PID gains

### Problem: Goes to FAULT immediately

**Check:**
- `Shooter/FaultMessage` for reason
- Motor current (overcurrent fault)
- Hood angle (out of range fault)
- Timeout (spinup or aiming took too long)

### Problem: Motors spin wrong direction

**Solutions:**
- Check `FLYWHEEL_MOTOR_INVERTED` in ShooterConstants.java
- Check `HOOD_WHEEL_MOTOR_INVERTED` in ShooterConstants.java
- Verify physical motor orientation

### Problem: Hood angle doesn't match expected

**Solutions:**
- Call `resetHoodEncoder(currentAngle)` at startup
- Check `HOOD_ANGLE_GEAR_RATIO` is correct
- Verify encoder is reading correctly

### Problem: D-Pad doesn't work in simulation

**This is expected!** The D-Pad (POV) often doesn't work in WPILib simulation.

**Solutions:**
- Use **Right Stick UP/DOWN** for hood angle control
- Use **Left Stick LEFT/RIGHT** for direction control
- D-Pad should work on real hardware

### Problem: Controller not detected

**Solutions:**
- Plug in controller **before** starting simulation
- Check "System Joysticks" panel shows "0: Controller"
- Try unplugging and replugging the controller
- Restart simulation

### Problem: Button presses not registering

**Solutions:**
- Make sure robot is **Enabled** (click "Teleoperated")
- Check console for "[CONTROLLER] Button detected!" messages
- Verify controller is assigned to port 0

---

## 8. Test Checklist

### Pre-Test Checklist

- [ ] Test mode enabled in Robot.java
- [ ] Code builds without errors
- [ ] Controller connected (if using)
- [ ] Dashboard open and showing values
- [ ] (Hardware only) Area clear, safety glasses on

### Simulation Test Checklist

- [ ] Simulation starts without errors
- [ ] "SHOOTER TEST MODE ACTIVE" message appears
- [ ] Automatic D-Pad test completes successfully
- [ ] Enable/Disable works (A button)
- [ ] Prepare shot works (X button)
- [ ] State transitions: IDLE → SPINNING_UP → AIMING → READY
- [ ] Direction toggle works (Y button)
- [ ] Manual RPM control works (RB/LB)
- [ ] Hood angle +5° works (Right Stick UP)
- [ ] Hood angle -5° works (Right Stick DOWN)
- [ ] Set LEFT direction works (Left Stick LEFT)
- [ ] Set RIGHT direction works (Left Stick RIGHT)
- [ ] Emergency stop works (Start)
- [ ] Fault clear works (Back)
- [ ] Shot simulation works (B button)

### Hardware Test Checklist

- [ ] Deploy successful
- [ ] Driver Station connected
- [ ] Flywheel motor spins
- [ ] Hood motor moves
- [ ] Hood wheel motor spins
- [ ] Direction affects motor directions
- [ ] RPM reaches target
- [ ] Angle reaches target
- [ ] ReadyToShoot becomes true
- [ ] Emergency stop works
- [ ] No unexpected faults

### Sign-Off

| Test | Passed | Tester | Date |
|------|--------|--------|------|
| Simulation basic | | | |
| Simulation full sequence | | | |
| Hardware basic | | | |
| Hardware full sequence | | | |

---

## Reverting to Normal Mode

After testing, remember to change Robot.java back:

```java
// Line 19
private final RobotContainer m_robotContainer;

// Line 28
m_robotContainer = new RobotContainer();
```

Then rebuild:
```bash
./gradlew build
```

---

*Document created for Team 6059 ShooterSubsystem testing*
