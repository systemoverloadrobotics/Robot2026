# FRC Team 6059 — Robot2026 Project Documentation

> **Audience:** New students, mentors, developers, technical support & implementation teams
> **Last Updated:** 2026-02-20
> **Season:** 2026 FRC
> **Language:** Java 17

---

## Table of Contents

1. [What is FRC? (Start Here if You Are New)](#1-what-is-frc-start-here-if-you-are-new)
2. [How a Robot Program Works — The Big Picture](#2-how-a-robot-program-works--the-big-picture)
3. [What Does Our Robot Do?](#3-what-does-our-robot-do)
4. [Getting Started — Setting Up Your Computer](#4-getting-started--setting-up-your-computer)
5. [Project Structure — Where is Everything?](#5-project-structure--where-is-everything)
6. [Core Concepts You Need to Know](#6-core-concepts-you-need-to-know)
   - 6.1 [What is a Subsystem?](#61-what-is-a-subsystem)
   - 6.2 [What is a Command?](#62-what-is-a-command)
   - 6.3 [What is PID Control?](#63-what-is-pid-control)
   - 6.4 [What is CAN Bus?](#64-what-is-can-bus)
   - 6.5 [What are Motors and Encoders?](#65-what-are-motors-and-encoders)
   - 6.6 [What are Constants?](#66-what-are-constants)
   - 6.7 [Open-Loop vs Closed-Loop Control](#67-open-loop-vs-closed-loop-control)
7. [Robot Lifecycle — What Runs and When](#7-robot-lifecycle--what-runs-and-when)
8. [Our Subsystems — Detailed Breakdown](#8-our-subsystems--detailed-breakdown)
   - 8.1 [Shooter (ShooterSubSystem)](#81-shooter-shootersubsystem)
   - 8.2 [Storage](#82-storage)
   - 8.3 [Intake (IntakeSubsystem)](#83-intake-intakesubsystem)
   - 8.4 [Climb](#84-climb)
9. [ShooterCalculator — The Math Behind Shooting](#9-shootercalculator--the-math-behind-shooting)
10. [Controller Buttons — What Does Each Button Do?](#10-controller-buttons--what-does-each-button-do)
11. [Hardware Map — Every Motor and Sensor on the Robot](#11-hardware-map--every-motor-and-sensor-on-the-robot)
12. [Constants Reference — All the Magic Numbers](#12-constants-reference--all-the-magic-numbers)
13. [Third-Party Libraries We Use](#13-third-party-libraries-we-use)
14. [Building, Deploying, and Simulating](#14-building-deploying-and-simulating)
15. [What is Done vs. What Still Needs Work](#15-what-is-done-vs-what-still-needs-work)
16. [Troubleshooting — When Things Go Wrong](#16-troubleshooting--when-things-go-wrong)
17. [Complete Glossary — Every Term Explained](#17-complete-glossary--every-term-explained)

---

## 1. What is FRC? (Start Here if You Are New)

**FRC (FIRST Robotics Competition)** is a worldwide high school robotics competition. Each January a new game is announced, and teams have a build season to design, build, and program a robot to compete in that game.

Here is what you need to know:

- **The Robot** — A ~120 lb machine with motors, sensors, wheels, and a central computer called the **RoboRIO** that runs our code.
- **The Code** — Written in **Java** (a programming language). Our code tells the robot what to do: how fast to spin a motor, what angle to set a mechanism, when to pick up or shoot game pieces.
- **The Match** — A match has two phases:
  - **Autonomous (auto):** The first ~15 seconds. The robot drives itself using pre-programmed instructions. No human input allowed.
  - **Teleoperated (teleop):** The remaining ~2 minutes. A human driver controls the robot using an **Xbox controller**.
- **The Driver Station** — A laptop running special software that connects to the robot over Wi-Fi. This is where drivers enable/disable the robot and view status information.
- **The Team** — We are **Team 6059 (System Overload Robotics)**.

### What Programming Language Do We Use?

We use **Java 17**. If you have never used Java before, that is completely fine — you will learn as you go. Java is a popular programming language used in many industries. FRC is a great way to learn it.

### What Framework Do We Use?

We use **WPILib**, which is a free toolkit made specifically for FRC robots. WPILib gives us:
- Ready-made classes for talking to motors, sensors, and controllers
- A structure called **"command-based programming"** that organizes our code (explained in detail in Section 6)
- Tools to build, deploy, and simulate our robot code

---

## 2. How a Robot Program Works — The Big Picture

Think of the robot code like a video game running at **50 frames per second**. Every 20 milliseconds (0.02 seconds), the robot:

1. **Reads inputs** — What buttons is the driver pressing? What do the sensors say?
2. **Makes decisions** — Based on inputs, what should each mechanism do?
3. **Sends outputs** — Tells each motor how fast to spin or what position to hold.

This happens in a continuous loop, 50 times per second, for the entire match.

### The Three Most Important Files

| File | What It Does | Analogy |
|---|---|---|
| `Main.java` | Starts the program. You never need to change this. | The power button |
| `Robot.java` | Manages the game phases (auto, teleop, disabled). Runs the main loop. | The game engine |
| `RobotContainer.java` | Creates all the mechanisms, wires up the Xbox controller buttons, and contains the main logic. | The game level designer |

### How They Connect

```
Main.java  (starts everything)
  └── Robot.java  (runs the 50 Hz loop, manages game phases)
        └── RobotContainer.java  (creates mechanisms, assigns buttons)
              ├── Shooter       ← the shooting mechanism
              ├── Storage       ← the conveyor belt
              ├── Intake        ← the pickup mechanism
              ├── Climb         ← the climbing mechanism
              └── Xbox Controller ← the driver's gamepad
```

---

## 3. What Does Our Robot Do?

Our robot has **four main mechanisms** (called **subsystems** in code):

### 1. Shooter — Launches game pieces ("fuel") at a target
Think of a baseball pitching machine. Two spinning wheels (called **flywheels**) grip the game piece and launch it. An adjustable ramp on top (called the **hood**) controls the launch angle — steeper for close shots, flatter for far shots.

### 2. Storage — Moves fuel inside the robot
A conveyor belt with rollers that moves fuel from the intake up to the shooter. It can also reverse to spit fuel back out.

### 3. Intake — Picks up fuel from the ground
A spinning roller on a movable arm. The arm swings down to reach the ground, the roller spins to grab the fuel, then the arm swings back up. Think of it like a hand reaching down to grab a ball.

### 4. Climb — Lifts the robot off the ground
At the end of a match, robots can earn bonus points by climbing onto a field structure. This mechanism extends an arm upward, hooks onto the bar, and pulls the robot up.

### Two Shooting Modes

The shooter has two modes the driver can switch between:

- **MANUAL mode** — The driver controls the flywheel speed by hand using the controller trigger and bumpers. Good for testing and experimentation.
- **AUTO mode** — The robot automatically calculates the best flywheel speed and hood angle based on distance. When everything is aimed correctly, it automatically feeds the fuel into the shooter.

---

## 4. Getting Started — Setting Up Your Computer

### Step 1: Install the Required Software

| Software | What It Is | Where to Get It |
|---|---|---|
| **WPILib 2026** | The FRC development toolkit. Includes VS Code (the code editor), Java, and build tools. | [wpilib.org](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) |
| **Git** | Version control — lets multiple people work on the same code without overwriting each other's work. Think of it like Google Docs but for code. | [git-scm.com](https://git-scm.com/) |
| **Phoenix Tuner X** | A tool from CTRE (the motor company) for configuring and testing motors. | [store.ctr-electronics.com](https://store.ctr-electronics.com/software/) |
| **PathPlanner** | A visual tool for drawing autonomous paths. Optional. | Available in VS Code extensions |

### Step 2: Get the Code

Open a terminal (or Git Bash on Windows) and run:

```bash
git clone https://github.com/systemoverloadrobotics/Robot2026.git
cd Robot2026
```

This **clones** (downloads) our code onto your computer.

### Step 3: Build the Code

```bash
./gradlew build
```

> **What is Gradle?** Gradle is a build tool. When you type `./gradlew build`, it compiles (translates) our Java code into something the robot's computer can run. It also downloads any libraries we depend on. You do not need to understand Gradle deeply — just know that this command builds the project.

If the build succeeds, you should see `BUILD SUCCESSFUL`. If it fails, ask a mentor for help with the error message.

### Step 4: Deploy to the Robot

Make sure you are connected to the robot (USB cable or the robot's Wi-Fi network), then:

```bash
./gradlew deploy
```

This sends the compiled code to the **RoboRIO** (the robot's brain computer). The robot's IP address is `10.60.59.2` (based on our team number 6059).

### Step 5: Simulate Without a Robot

If you do not have access to the physical robot:

```bash
./gradlew simulateJava
```

This opens a simulation window where you can test your code on your laptop. Motors and sensors are simulated — not perfect, but great for checking logic.

### Git Branches

Our code has different **branches** — think of them as parallel versions of the code:

| Branch | What It Is |
|---|---|
| `main` | The stable, tested version. This is what we use at competitions. |
| `Climb` | Where we are developing the climbing feature. |

To switch branches:
```bash
git checkout main        # Switch to the main branch
git checkout Climb       # Switch to the Climb branch
```

---

## 5. Project Structure — Where is Everything?

Here is every important file in the project, with plain-English descriptions:

```
Robot2026/
│
├── build.gradle                     # Build instructions (like a recipe for compiling the code)
├── settings.gradle                  # Tells Gradle where to find plugins
├── tuner-swerve-project.json        # Configuration for swerve drive (not yet in use)
│
├── .wpilib/
│   └── wpilib_preferences.json      # Our team number (6059) and project settings
│
├── vendordeps/                      # Third-party libraries (like apps on your phone)
│   ├── AdvantageKit.json            # Advanced data recording tool
│   ├── DogLog.json                  # Simple data recording tool (we use this one)
│   ├── PathplannerLib-2026.1.2.json # Autonomous path planning
│   ├── Phoenix6-26.1.0.json         # Motor and sensor control library
│   └── WPILibNewCommands.json       # The command-based programming framework
│
├── src/main/java/frc/robot/         # ALL OF OUR CODE LIVES HERE
│   │
│   ├── Main.java                    # Entry point — just starts the program. Never edit this.
│   ├── Robot.java                   # Manages game phases (auto, teleop, etc.)
│   ├── RobotContainer.java          # THE MAIN FILE — creates mechanisms, assigns buttons
│   ├── Constants.java               # All the numbers (motor IDs, speeds, PID values)
│   │
│   ├── commands/                    # Actions the robot can perform
│   │   ├── Autos.java               # Autonomous routines (template only right now)
│   │   └── ExampleCommand.java      # Example template (unused)
│   │
│   ├── subsystems/                  # One file per mechanism
│   │   ├── ShooterSubSystem.java    # The shooter (flywheels + hood)
│   │   ├── Storage.java             # The conveyor belt
│   │   ├── IntakeSubsystem.java     # The ground pickup
│   │   ├── Climb.java               # The climbing arm
│   │   └── ExampleSubsystem.java    # Example template (unused)
│   │
│   └── utils/                       # Helper tools
│       └── ShooterCalculator.java   # Math for calculating shot speed and angle
│
└── src/main/deploy/                 # Files that get copied to the robot
    └── pathplanner/                 # Autonomous path data
        ├── navgrid.json             # Map of the field with obstacles
        └── paths/
            └── Example Path.path    # A sample autonomous path
```

### Which Files Will I Edit Most?

As a new student, you will most likely work in:

1. **`RobotContainer.java`** — Adding new button bindings or changing how mechanisms work together
2. **`Constants.java`** — Changing motor IDs, speeds, or tuning values
3. **Files in `subsystems/`** — Adding new features to a mechanism
4. **Files in `commands/`** — Creating new actions for the robot

---

## 6. Core Concepts You Need to Know

### 6.1 What is a Subsystem?

A **subsystem** is a Java class that represents one physical mechanism on the robot. Each subsystem "owns" its motors and sensors — no other code is allowed to directly control them.

**Why?** Imagine two pieces of code both trying to control the same motor at the same time — one says "spin forward" and the other says "spin backward." The motor would jitter or break. Subsystems prevent this by ensuring only one piece of code controls a mechanism at a time.

**Our subsystems:**
| Subsystem Class | Physical Mechanism | Motors It Owns |
|---|---|---|
| `ShooterSubSystem` | Flywheel launcher + adjustable hood | 3 motors + 1 encoder |
| `Storage` | Conveyor belt rollers | 1 motor |
| `IntakeSubsystem` | Ground pickup with pivoting arm | 2 motors + 1 encoder |
| `Climb` | Telescoping climbing arm | 1 motor |

### 6.2 What is a Command?

A **command** is an action that uses one or more subsystems. Think of it like giving the robot an instruction:

- "Spin the shooter wheels at 50 ft/s" — a command using the Shooter subsystem
- "Run the conveyor forward" — a command using the Storage subsystem
- "Extend the climb arm, then retract it" — a sequence of commands using the Climb subsystem

Commands can be:
- **Instant** — Do something once (like "set this speed")
- **Continuous** — Keep doing something while a button is held (like "keep spinning while I hold the trigger")
- **Sequential** — Do step 1, then step 2, then step 3
- **Conditional** — Only do this if a certain condition is true

In our code, we create commands using helper methods like:
- `Commands.runOnce(() -> ...)` — Run a piece of code exactly once
- `Commands.run(() -> ...)` — Run a piece of code repeatedly (every 20 ms)
- `Commands.runEnd(() -> startAction, () -> stopAction)` — Run one action while active, then a cleanup action when done

### 6.3 What is PID Control?

**PID** stands for **Proportional, Integral, Derivative**. It is a way to precisely control a motor to reach and hold a target value (like a specific speed or position).

**The Problem PID Solves:**

Imagine you want the shooter wheel spinning at exactly 50 ft/s. You cannot just say "give the motor 80% power" because:
- If the battery is fully charged, 80% power might give you 55 ft/s (too fast)
- If the battery is low, 80% power might give you 40 ft/s (too slow)
- If a game piece enters the shooter, it might slow down the wheel

PID continuously measures the difference between where you ARE and where you WANT TO BE (the **error**), and adjusts the motor power automatically.

**The Three Parts:**

| Letter | Name | What It Does | Real-World Analogy |
|---|---|---|---|
| **P** | Proportional | Reacts to the current error. Bigger error = bigger correction. | You are far from your goal, so you push harder. |
| **I** | Integral | Reacts to accumulated past error. Fixes small persistent errors over time. | You have been slightly off for a while, so you nudge a bit more. |
| **D** | Derivative | Reacts to how fast the error is changing. Prevents overshooting. | You are approaching your goal quickly, so you ease off to avoid zooming past it. |

**The Values (Gains):**

Each part has a number called a **gain** that determines how strongly it reacts:
- **kP** = Proportional gain (most important, start here when tuning)
- **kI** = Integral gain (often set to 0 in FRC)
- **kD** = Derivative gain (helps with oscillation)

**Tuning** is the process of finding the right kP, kI, and kD values. This is done by testing on the real robot and adjusting until the mechanism moves smoothly to its target without overshooting or oscillating.

**Feedforward (kV):**

Some of our motors also use **kV** (velocity feedforward). This is a prediction: "To spin at X speed, I probably need about Y voltage." PID then handles the fine-tuning on top of that prediction. Using feedforward makes PID's job much easier.

### 6.4 What is CAN Bus?

**CAN bus** (Controller Area Network) is how all the electronic devices on the robot communicate. Think of it as a single cable (actually two wires twisted together — yellow and green) that daisy-chains from device to device.

```
RoboRIO ──── Motor 1 ──── Motor 2 ──── Sensor 1 ──── Motor 3 ──── ...
              (yellow/green wires connecting each device in a chain)
```

**Every device on the CAN bus has a unique ID number** (like an address). When the code says "Motor 13, spin at 50 ft/s," the message travels along the CAN wire until it reaches the device with ID 13.

**Our CAN IDs:**
- Storage roller: **1**
- Hood pivot motor: **11**
- Top flywheel: **13**
- Bottom flywheel: **14**
- Hood encoder: **15**
- Intake pivot: **31**
- Intake roller: **32**
- Intake encoder: **33**
- Climb motor: **not yet assigned**

**Important:** If two devices have the same CAN ID, neither will work correctly. Each ID must be unique. You set CAN IDs using **Phoenix Tuner X** software.

### 6.5 What are Motors and Encoders?

**Motors** spin things. In FRC, we primarily use:

- **TalonFX** — This is a motor controller made by CTRE (Cross The Road Electronics). It is built directly into the **Falcon 500** and **Kraken X60** motors. It has a built-in encoder and communicates over CAN bus. Our robot has **7 TalonFX motors**.

**Encoders** measure rotation — how far and how fast something has turned. There are two types on our robot:

- **Built-in motor encoders** — Every TalonFX has one inside. It measures how many times the motor shaft has spun. Very precise, but it does not know where "zero" is when the robot first turns on (it starts counting from wherever it happens to be).
- **CANcoders** — External absolute encoders made by CTRE. These know their exact position even after the robot is powered off and on again, because they use a magnet. We use them on the shooter hood and intake pivot so the robot always knows the exact angle of these mechanisms. Our robot has **2 CANcoders** (IDs 15 and 33).

**FusedCANcoder** is a special mode that combines the best of both: the absolute position knowledge of the CANcoder with the high-speed updates of the motor's built-in encoder.

### 6.6 What are Constants?

**Constants** are numbers that do not change while the code is running, but we might want to adjust between test runs. Instead of scattering these numbers throughout the code (making them hard to find and change), we put them all in one file: `Constants.java`.

Examples of constants:
- Motor CAN IDs (which motor is plugged into which port)
- PID gains (kP, kI, kD values)
- Speed limits (how fast a roller should spin)
- Physical measurements (wheel diameter, gear ratios)

**Why this matters:** If you need to change a motor ID or adjust a speed, you only need to look in `Constants.java` — you do not have to search through every file.

### 6.7 Open-Loop vs Closed-Loop Control

These are two fundamentally different ways to control a motor:

**Open-Loop (DutyCycleOut):**
- You tell the motor "run at 80% power"
- The motor gives 80% power, regardless of what actually happens
- Simple, but imprecise — the actual speed depends on battery voltage, load, friction, etc.
- We use this for **simple mechanisms** like the storage rollers and intake roller, where exact speed does not matter much

**Closed-Loop (VelocityVoltage, PositionVoltage):**
- You tell the motor "reach and hold 50 ft/s" or "go to position 30 degrees"
- The motor uses PID to continuously measure and adjust its output to hit that target
- More precise, but requires tuning the PID gains
- We use this for the **shooter flywheel** (exact speed matters for consistent shots) and **hood angle** (exact position matters for aim)

| Control Type | Code Name | What You Set | Used For |
|---|---|---|---|
| Open-loop | `DutyCycleOut` | Percent power (-1.0 to 1.0) | Storage rollers, Intake roller |
| Closed-loop velocity | `VelocityVoltage` | Target speed (ft/s or RPM) | Shooter flywheel |
| Closed-loop position | `PositionVoltage` | Target position (degrees or rotations) | Hood angle, Intake pivot, Climb |

---

## 7. Robot Lifecycle — What Runs and When

**File:** `src/main/java/frc/robot/Robot.java`

An FRC match has several phases. The robot code has special methods that run during each phase:

### Match Timeline

```
┌─────────────────────────────────────────────────────────────┐
│  DISABLED  │  AUTONOMOUS  │  DISABLED  │  TELEOPERATED      │
│ (waiting)  │  (~15 sec)   │  (brief)   │  (~2 min 15 sec)   │
│            │  Robot acts  │            │  Driver controls    │
│            │  on its own  │            │  the robot          │
└─────────────────────────────────────────────────────────────┘
```

### What Each Method Does

| Method | When It Runs | What It Does In Our Code |
|---|---|---|
| `robotInit()` | Once, when the robot turns on | Creates all subsystems (Shooter, Storage, Intake, Climb) and sets up Xbox controller bindings |
| `robotPeriodic()` | Every 20 ms, **always** (even when disabled) | Runs the command scheduler (executes all active commands). Also calls `updateShooter()` to manage auto-shooting |
| `autonomousInit()` | Once, when autonomous starts | Starts the match timer |
| `autonomousPeriodic()` | Every 20 ms during autonomous | (Currently empty — autonomous routines not yet implemented) |
| `teleopInit()` | Once, when teleop starts | (Currently empty — controller bindings are set up in `robotInit`) |
| `teleopPeriodic()` | Every 20 ms during teleop | (Currently empty — commands handle everything) |
| `disabledInit()` | Once, when robot is disabled | (Currently empty) |
| `disabledPeriodic()` | Every 20 ms while disabled | (Currently empty) |

**Key Detail:** `robotPeriodic()` runs during ALL phases, including disabled. This is where the **command scheduler** lives — it is the heart of the robot's control system. It checks which commands are active and runs them every cycle.

### The Command Scheduler — The Brain

The **command scheduler** is like a task manager. Every 20 ms it:

1. Checks if any buttons were pressed → starts new commands
2. Runs all currently active commands
3. Checks if any commands are finished → cleans them up
4. Makes sure no two commands try to use the same subsystem

You do not write the scheduler — WPILib provides it. You just create commands and bind them to buttons.

---

## 8. Our Subsystems — Detailed Breakdown

### 8.1 Shooter (ShooterSubSystem)

**File:** `src/main/java/frc/robot/subsystems/ShooterSubSystem.java`

#### What It Does

The shooter launches game pieces at a target. It works like a baseball pitching machine:

```
                ┌── Hood (adjustable ramp) ──┐
                │    sets the launch angle   │
                └────────────────────────────┘
                         ▲
    Game piece           │
    enters here ──►  ╔═══╗  ──► Game piece
                     ║ ○ ║      launches out
                     ║   ║
                     ║ ○ ║
                     ╚═══╝
                  Two spinning
                  flywheel wheels
                  (top and bottom)
```

The **flywheels** spin at high speed. When a game piece is pushed between them, the spinning wheels grab it and fling it out. The **hood** is an adjustable ramp at the top that tilts to change the launch angle.

#### Hardware (What is Physically on the Robot)

| Part | Motor/Sensor Type | CAN ID | What It Does |
|---|---|---|---|
| Top Flywheel | TalonFX motor | 13 | Spins the top wheel |
| Bottom Flywheel | TalonFX motor | 14 | Spins the bottom wheel (automatically follows the top motor in the opposite direction) |
| Hood Pivot | TalonFX motor | 11 | Tilts the hood up or down |
| Hood Encoder | CANcoder | 15 | Tells the code exactly what angle the hood is at |

#### How the Flywheel Control Works

The flywheels use **closed-loop velocity control** (see Section 6.7):

- You tell the code: "I want the flywheel spinning at 50 feet per second"
- The code converts that to rotations per second using the wheel size (4 inch diameter, 1.047 ft circumference)
- The PID controller on the motor adjusts power to reach and hold that speed
- The code considers the flywheel "ready" when it is within **2.0 ft/s** of the target (the tolerance)

**PID values used:** kP = 0.12448, kV = 0.12 (feedforward) — these have been tuned and tested.

**Voltage limits:** The motor is limited to 9 volts (out of a max ~12V) to prevent brownouts (when too many motors draw too much power at once and the battery voltage drops).

#### How the Hood Angle Control Works

The hood uses **closed-loop position control**:

- You tell the code: "I want the hood at 30 degrees"
- The code uses the CANcoder (absolute encoder) to know the current angle
- PID adjusts the motor to move the hood to that angle and hold it there
- The hood motor is in **brake mode** — when not actively moving, it holds its position instead of freewheeling

**Gear ratio:** 50:1. This means the motor must spin 50 times for the hood to rotate once. This gives very precise control but slow movement.

**Safety limits:** The hood is clamped to 10°–170° in code to prevent it from hitting mechanical stops.

**PID values used:** kP = 0.05, kD = 0.001 — tuned to move smoothly without oscillating.

#### Angle Ranges by Side

The shooter can aim to the left or right side:

| Side | Starting Angle | Allowed Range |
|---|---|---|
| LEFT | 30° | 15° to 45° |
| RIGHT | 150° | 135° to 165° |

#### Available Functions (Public API)

These are the methods you can call from other parts of the code:

| Method | What You Pass In | What It Does |
|---|---|---|
| `setFlywheelVelocity(speed)` | A speed in ft/s (e.g., `FeetPerSecond.of(50)`) | Sets the target flywheel speed |
| `setHoodAngle(angle)` | An angle in degrees (e.g., `Degrees.of(30)`) | Sets the target hood angle |
| `getFlywheelVelocity()` | Nothing | Returns the current flywheel speed |
| `getHoodAngle()` | Nothing | Returns the current hood angle |
| `isFlywheelAtTarget()` | Nothing | Returns `true` if the flywheel is within 2 ft/s of its target |
| `isHoodAngleAtTarget()` | Nothing | Returns `true` if the hood is within 1.5° of its target |
| `isAtTarget()` | Nothing | Returns `true` if BOTH flywheel and hood are at their targets |

#### Telemetry (Data Logging)

The shooter logs data using **DogLog** so you can view it later for debugging:

- `Shooter/FlywheelVelocity` — How fast the flywheel is actually spinning
- `Shooter/FlywheelTargetVelocity` — How fast we want it to spin
- `Shooter/HoodAngle` — Where the hood actually is
- `Shooter/HoodTargetAngle` — Where we want the hood to be

You can view these values in **AdvantageScope** (a data viewer application).

---

### 8.2 Storage

**File:** `src/main/java/frc/robot/subsystems/Storage.java`

#### What It Does

The storage is a **conveyor belt** inside the robot. It moves game pieces ("fuel") from the intake to the shooter. Think of it like the conveyor belt at a grocery store checkout — it moves items from one end to the other.

```
        ┌──────────────────────┐
Fuel    │  ○ ○ ○ ○ ○ ○ ○ ○ ○  │  ──► To Shooter
enters  │  Rollers spin to     │
here    │  move fuel along     │
        └──────────────────────┘
```

#### Hardware

| Part | Type | CAN ID | What It Does |
|---|---|---|---|
| Roller Motor | TalonFX | 1 | Drives the conveyor rollers |

#### How It Works

This is one of the simplest subsystems. It uses **open-loop control** (just set a power percentage):

| State | Power Level | What Happens |
|---|---|---|
| `FORWARD` | 80% power | Moves fuel toward the shooter |
| `REVERSE` | 50% power backward | Moves fuel away from the shooter (for ejecting) |
| `OFF` | 0% | Stops the rollers |

**Brake mode** is enabled — when the rollers are set to `OFF`, the motor actively stops instead of coasting to a halt. This prevents fuel from sliding around.

**Current limit:** 35 amps. This protects the motor from drawing too much current if a game piece gets jammed.

#### Available Functions

| Method | What You Pass In | What It Does |
|---|---|---|
| `setRollers(state)` | `RollerState.FORWARD`, `RollerState.REVERSE`, or `RollerState.OFF` | Sets the rollers to the given state |

---

### 8.3 Intake (IntakeSubsystem)

**File:** `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`

#### What It Does

The intake picks up game pieces from the ground. It has two parts:

1. **A roller** — Spins to grab the game piece
2. **A pivot arm** — Swings the roller down to reach the ground, then back up

```
    Robot frame
    ┌──────────┐
    │          │
    │    ╱     │    Arm swings down
    │   ╱      │    to reach ground
    │  ● roller│
    └──────────┘
    ═══════════════  Ground
```

#### Hardware

| Part | Type | CAN ID | CAN Bus | What It Does |
|---|---|---|---|---|
| Roller Motor | TalonFX | 32 | rio | Spins the roller to grab game pieces |
| Pivot Motor | TalonFX | 31 | rio | Swings the arm up and down |
| Pivot Encoder | CANcoder | 33 | rio | Tells the code exactly what angle the arm is at |

> **"CAN Bus: rio"** — Our robot has more than one CAN network. The intake motors are on the one labeled "rio" (the one directly connected to the RoboRIO).

#### How It Works

- **Roller:** Open-loop control. Just set a power level (e.g., -0.5 for outtake/ejecting).
- **Pivot:** Closed-loop position control using PID. You tell it what angle to go to, and PID moves the motor to that angle.

**Important:** When the robot turns on, the code reads the CANcoder's absolute position and sets the motor's internal encoder to match. This is called **encoder syncing** — it means the motor always knows exactly where the arm is, even after a reboot.

#### Current Status

**The intake pivot PID is NOT tuned yet.** The kP, kI, and kD values are all set to 0, which means the pivot motor will not actually move to a target position. These values need to be determined through testing on the real robot.

#### Available Functions

| Method | What You Pass In | What It Does |
|---|---|---|
| `start()` | Nothing | Starts the roller at default power |
| `stop()` | Nothing | Stops the roller |
| `setPower(power)` | A number from -1.0 to 1.0 | Sets the roller speed. Positive = intake, Negative = eject |
| `setPivotPosition(position)` | A number (in rotations) | Moves the arm to a specific position |
| `setPivotPosition(angle)` | An angle (e.g., `Degrees.of(45)`) | Moves the arm to a specific angle |

---

### 8.4 Climb

**File:** `src/main/java/frc/robot/subsystems/Climb.java`

#### What It Does

At the end of a match, robots can earn bonus points by climbing onto a field structure. The climb mechanism extends an arm upward, hooks onto a bar, and then retracts to pull the robot off the ground.

```
    ┌─ Bar on field ─┐
    │                │
    │   ↑ Arm grabs  │
    │   │ the bar     │
    ┌───┴───┐
    │ Robot │         Arm retracts
    │       │         to pull robot up
    └───────┘
```

#### Hardware

| Part | Type | CAN ID | CAN Bus | What It Does |
|---|---|---|---|---|
| Climb Motor | TalonFX | **NOT SET (-1)** | rio | Extends and retracts the climb arm |

#### Current Status

**This subsystem is NOT ready for use.** All of its configuration values are placeholder (-1):

- Motor CAN ID: not assigned
- PID gains: not tuned
- Gear ratio: not measured
- Current limit: not set
- Extension/retraction positions: not measured

Before this subsystem can be used, someone needs to:
1. Physically install the motor and assign it a CAN ID using Phoenix Tuner X
2. Measure the gear ratio (how many motor rotations = one mechanism rotation)
3. Find the correct extension and retraction positions by manually moving the mechanism
4. Tune the PID values by testing on the robot
5. Set an appropriate current limit to protect the motor

#### Available Functions (will work once configured)

| Method | What You Pass In | What It Does |
|---|---|---|
| `extendArm()` | Nothing | Extends the arm to max height |
| `retractArm()` | Nothing | Retracts the arm fully |
| `climbAndHold()` | Nothing | Moves to a holding position |
| `climbAndHold(position)` | A number (rotations) | Moves to a custom holding position |
| `isExtended()` | Nothing | Returns `true` if the arm is fully extended |
| `isRetracted()` | Nothing | Returns `true` if the arm is fully retracted |
| `isAtHoldPosition()` | Nothing | Returns `true` if the arm is at the hold position |

---

## 9. ShooterCalculator — The Math Behind Shooting

**File:** `src/main/java/frc/robot/utils/ShooterCalculator.java`

### What Problem Does This Solve?

When shooting at a target from different distances, you need different flywheel speeds and hood angles. A shot from 5 feet away needs a different speed and angle than a shot from 15 feet away.

Rather than manually testing and hardcoding every possible distance, we use **math** to predict the right values for any distance.

### How It Works — Regression (Primary Method)

We took the robot to the practice field and tested shots at many different distances. For each distance, we recorded what flywheel speed and hood angle made the best shot. Then we used a tool called **regression** (available on [Desmos](https://www.desmos.com/calculator/wune2ctiat)) to find a mathematical formula (a curve) that fits all those data points.

The formula is a **4th-degree polynomial** — it looks like this:

```
velocity = va*x^4 + vb*x^3 + vc*x^2 + vd*x + vf
angle    = aa*x^4 + ab*x^3 + ac*x^2 + ad*x + af
```

Where `x` is the distance in **inches**, and the `va`, `vb`, etc. are numbers (coefficients) that were calculated by the regression tool.

**In plain English:** "Give me a distance, and I will calculate the best flywheel speed and hood angle using a formula."

### Lookup Tables (Secondary Method)

An alternative approach. Instead of a formula, you store a list of tested values:

```
At  2 feet → use speed X, angle Y
At  4 feet → use speed X, angle Y
At  6 feet → use speed X, angle Y
...
```

For distances between entries, the code estimates using **linear interpolation** (drawing a straight line between the two nearest known points).

**Current status:** The lookup tables are **empty** (all zeros). They need to be filled in with real test data if you want to use this method instead of regression.

### Available Functions

| Method | What You Pass In | What You Get Back | Description |
|---|---|---|---|
| `getRegressionVelocity(distance)` | A distance (e.g., `Feet.of(10)`) | A speed in ft/s | Best flywheel speed for that distance (formula) |
| `getRegressionAngle(distance)` | A distance | An angle in degrees | Best hood angle for that distance (formula) |
| `getLookupTableVelocity(distance)` | A distance | A speed in ft/s | Flywheel speed from the table (currently empty) |
| `getLookupAngle(distance)` | A distance | An angle in degrees | Hood angle from the table (currently empty) |

---

## 10. Controller Buttons — What Does Each Button Do?

We use a standard **Xbox controller** connected to the Driver Station laptop via USB (port 0).

```
         LB          RB
        ┌──┐        ┌──┐
     LT │  │        │  │ RT
    ┌───┴──┴────────┴──┴───┐
    │         (Y)          │
    │       (X) (B)        │
    │         (A)          │
    │                      │
    │   [LS]      [RS]     │
    └──────────────────────┘
    LB = Left Bumper    RB = Right Bumper
    LT = Left Trigger   RT = Right Trigger
```

### Current Button Assignments

| Button | When You Press/Release | What Happens | Which Mode |
|---|---|---|---|
| **Y** | When you **release** the button | Toggles between MANUAL and AUTO shooting mode | Works in both modes |
| **Left Trigger** | **Hold** it down | Spins up the shooter flywheel to the target speed | MANUAL only |
| **Left Trigger** | **Release** it | Stops the flywheel | MANUAL only |
| **Left Bumper** | Press once | Decreases target flywheel speed by 2 ft/s | MANUAL only |
| **Right Bumper** | Press once | Increases target flywheel speed by 2 ft/s | MANUAL only |

**Default flywheel speed:** 50 ft/s (adjustable with bumpers in 2 ft/s steps)

### Buttons NOT Yet Assigned

The following controller buttons are currently **unused** and available:
- A, B, X buttons
- Right Trigger
- Both joysticks (left stick, right stick)
- D-pad (up, down, left, right)
- Start, Back buttons

### Functions NOT Yet Assigned to Buttons

These are pieces of code that exist but have no button:

| Function | What It Would Do |
|---|---|
| `shuttleFuel()` | Reverse the storage + run intake outward (to eject fuel) |
| `stopShuttle()` | Stop the shuttle/eject operation |
| `shootFuel()` | Feed fuel into the shooter (only when flywheel and hood are ready) |
| Intake start/stop | Pick up fuel from the ground |
| Climb extend/retract | Extend and retract the climbing arm |

---

## 11. Hardware Map — Every Motor and Sensor on the Robot

### All CAN Devices

Every motor and sensor on the CAN bus, sorted by ID:

| CAN ID | Device Type | What It Is | Subsystem | CAN Bus |
|---|---|---|---|---|
| 1 | TalonFX (motor) | Storage conveyor roller | Storage | default |
| 11 | TalonFX (motor) | Shooter hood pivot | Shooter | default |
| 13 | TalonFX (motor) | Top flywheel | Shooter | default |
| 14 | TalonFX (motor) | Bottom flywheel | Shooter | default |
| 15 | CANcoder (sensor) | Hood angle sensor | Shooter | default |
| 31 | TalonFX (motor) | Intake pivot | Intake | rio |
| 32 | TalonFX (motor) | Intake roller | Intake | rio |
| 33 | CANcoder (sensor) | Intake pivot angle sensor | Intake | rio |
| **TBD** | TalonFX (motor) | Climb motor | Climb | rio |

### Motor Settings Summary

| Motor | What Happens When Power = 0 | Current Limit | How It Is Controlled |
|---|---|---|---|
| Top Flywheel (13) | Coasts to a stop | Max 9V | Closed-loop velocity (PID) |
| Bottom Flywheel (14) | Follows motor 13 | Max 9V | Follows motor 13 (opposite direction) |
| Hood Pivot (11) | Holds position (brake) | — | Closed-loop position (PID) |
| Storage Roller (1) | Holds still (brake) | 35 amps | Open-loop (percent power) |
| Intake Roller (32) | — | — | Open-loop (percent power) |
| Intake Pivot (31) | Holds position (brake) | 30 amps | Closed-loop position (PID) |
| Climb (TBD) | Holds position (brake) | TBD | Closed-loop position (PID) |

### Other Hardware

| Device | Port | What It Is |
|---|---|---|
| Xbox Controller | USB port 0 | The driver's gamepad |
| RoboRIO | — | The robot's main computer (runs our code) |
| Power Distribution Hub | — | Distributes battery power to all motors |
| Robot Radio | — | Wi-Fi connection between robot and Driver Station |

---

## 12. Constants Reference — All the Magic Numbers

**File:** `src/main/java/frc/robot/Constants.java`

All configurable numbers are stored here. If you need to change a motor ID, adjust a speed, or retune a PID loop, this is the file to edit.

### Operator Constants

| Name | Value | What It Means |
|---|---|---|
| `kDriverControllerPort` | 0 | The Xbox controller is plugged into USB port 0 |

### Shooter Constants

| Name | Value | What It Means |
|---|---|---|
| `TOP_FLYWHEEL_ID` | 13 | CAN ID of the top flywheel motor |
| `BOTTOM_FLYWHEEL_ID` | 14 | CAN ID of the bottom flywheel motor |
| `SHOOTER_PIVOT_ID` | 11 | CAN ID of the hood pivot motor |
| `SHOOTER_PIVOT_ENCODER` | 15 | CAN ID of the hood angle sensor |
| `kP` | 0.12448 | Flywheel PID: how aggressively it corrects speed errors |
| `kI` | 0.0 | Flywheel PID: accumulation correction (not used) |
| `kD` | 0.0 | Flywheel PID: dampening (not used) |
| `kV` | 0.12 | Flywheel feedforward: predicted voltage per unit of speed |
| `HOOD_KP` | 0.05 | Hood PID: how aggressively it corrects angle errors |
| `HOOD_KI` | 0.0 | Hood PID: accumulation correction (not used) |
| `HOOD_KD` | 0.001 | Hood PID: dampening to prevent oscillation |
| `FLYWHEEL_DIAMETER_INCHES` | 4.0 | Physical wheel diameter |
| `FLYWHEEL_CIRCUMFERENCE_FT` | 1.047 | Circumference of the wheel (pi * diameter, in feet) |
| `FLYWHEEL_GEAR_RATIO` | 1.0 | Motor directly drives wheel (no gears between them) |
| `FLYWHEEL_FPS_TOLERANCE` | 2.0 | "Close enough" threshold for flywheel speed (ft/s) |
| `SHOOTER_PIVOT_TOLERANCE` | 1.5 | "Close enough" threshold for hood angle (degrees) |
| `SHOOTER_PIVOT_GEAR_RATIO` | 50.0 | Motor spins 50 times for hood to spin once |

### Storage Constants

| Name | Value | What It Means |
|---|---|---|
| `ROLLER_MOTOR_ID` | 1 | CAN ID of the storage roller motor |
| `ROLLER_FORWARD_SPEED` | 0.8 | 80% power forward (feeding to shooter) |
| `ROLLER_REVERSE_SPEED` | -0.5 | 50% power backward (ejecting) |

### Intake Constants

| Name | Value | What It Means |
|---|---|---|
| `ROLLER_ID` | 32 | CAN ID of the intake roller motor |
| `PIVOT_ID` | 31 | CAN ID of the intake pivot motor |
| `ENCODER_ID` | 33 | CAN ID of the intake angle sensor |
| `KP, KI, KD` | 0, 0, 0 | Pivot PID gains — **ALL ZEROS, NOT TUNED** |
| `OuttakePower` | -0.5 | Roller speed when ejecting (50% backward) |
| `StartPower` | 0 | **Not configured** |
| `StopPower` | 0 | **Not configured** |

### Climb Constants

| Name | Value | What It Means |
|---|---|---|
| `CANBUS_RIO` | "rio" | Which CAN bus the motor is on |
| `MOTOR_ID` | -1 | **NOT SET** — needs a real CAN ID |
| `KP, KI, KD` | -1, -1, -1 | **NOT SET** — needs PID tuning |
| `SENSOR_TO_MECHANISM_RATIO` | -1 | **NOT SET** — needs gear ratio measurement |
| `SUPPLY_CURRENT_LIMIT` | -1 | **NOT SET** — needs a safe current limit |
| `MAX_EXTN_POSITION` | -1 | **NOT SET** — needs max extension position |
| `MIN_EXTN_POSITION` | -1 | **NOT SET** — needs min retraction position |
| `HOLD_POSITION` | -1 | **NOT SET** — needs hold position |
| `POSITION_TOLERANCE` | 0.5 | "Close enough" for climb position (0.5 rotations) |

---

## 13. Third-Party Libraries We Use

Our code depends on external libraries (code other people wrote that we use). These are defined in the `vendordeps/` folder.

| Library | Version | What It Does | Do We Currently Use It? |
|---|---|---|---|
| **Phoenix 6** | 26.1.0 | Made by CTRE. Lets us control TalonFX motors and CANcoders. Every subsystem depends on this. | Yes — essential |
| **DogLog** | 2026.2.0 | A simple logging library. Records data (like flywheel speed) so we can review it later. | Yes — in ShooterSubSystem |
| **AdvantageKit** | 26.0.0 | A more advanced logging and replay framework. Can record and replay entire matches for debugging. | Installed but not used yet |
| **PathPlanner** | 2026.1.2 | A tool for creating autonomous paths — the robot follows a drawn route on the field. | Installed, example path exists, but not used in code yet |
| **WPILib New Commands** | 1.0.0 | The command-based framework itself. This is what lets us write subsystems and commands. | Yes — the backbone of our code |

### How to Update Libraries

1. Open the project in VS Code (WPILib edition)
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type "WPILib: Manage Vendor Libraries" and select it
4. Choose "Check for updates (online)"
5. Follow the prompts to update

---

## 14. Building, Deploying, and Simulating

### What These Commands Do

| Command | What It Does | When to Use It |
|---|---|---|
| `./gradlew build` | Compiles the code and checks for errors | After making code changes, to verify they work |
| `./gradlew deploy` | Sends the compiled code to the robot | When you want to test on the real robot |
| `./gradlew simulateJava` | Runs the code on your computer with simulated hardware | When you do not have access to the robot |
| `./gradlew clean` | Deletes all compiled files (forces a fresh build next time) | When something is acting weird and you want a fresh start |
| `./gradlew downloadAll` | Downloads all required libraries | After cloning the project or updating vendor libraries |

### Deploying Step-by-Step

1. **Connect to the robot** — Either plug in a USB cable or connect to the robot's Wi-Fi network
2. **Open a terminal** in VS Code (Terminal → New Terminal)
3. **Run:** `./gradlew deploy`
4. **Wait** for it to compile and upload (30 seconds to 2 minutes)
5. **Open the Driver Station** on the driver laptop
6. The robot should show as connected. Enable it to test.

### What is the Deploy Directory?

The `src/main/deploy/` folder is special — anything inside it gets copied to the robot's file system at `/home/lvuser/deploy/`. This is used for data files the robot needs at runtime (like PathPlanner paths). You access these files in code with:

```java
Filesystem.getDeployDirectory()
```

---

## 15. What is Done vs. What Still Needs Work

This table tracks the status of every component. Green means ready, red means needs work.

| Component | Status | What Needs to Happen |
|---|---|---|
| Shooter flywheel PID | **DONE** | Tuned and tested (kP=0.12448, kV=0.12) |
| Shooter hood PID | **DONE** | Tuned and tested (kP=0.05, kD=0.001) |
| Shooter distance regression | **DONE** | 4th-degree polynomial curve fitted from test data |
| Storage rollers | **DONE** | Working with forward (80%) and reverse (50%) speeds |
| Intake roller | **DONE** | CAN IDs assigned, power values set |
| Intake pivot PID | **NEEDS TUNING** | PID gains are all 0 — need to test and find good values |
| Intake pivot positions | **NEEDS CONFIG** | No start/stop positions defined yet |
| Climb motor CAN ID | **NOT SET** | Need to install motor, assign CAN ID via Phoenix Tuner X |
| Climb PID gains | **NOT SET** | Need to tune after motor is installed |
| Climb positions | **NOT SET** | Need to measure extension, retraction, and hold positions |
| Climb current limit | **NOT SET** | Need to determine safe current limit |
| Climb gear ratio | **NOT SET** | Need to measure the mechanical gear ratio |
| Swerve drive (drivetrain) | **NOT STARTED** | Tuner config file exists but no code written yet |
| Autonomous routines | **NOT STARTED** | Only a template exists — need real auto paths |
| Vision system | **NOT STARTED** | Distance to target is hardcoded to 5 ft — need camera + vision code |
| Shooter lookup tables | **EMPTY** | Alternative to regression — tables have all zeros |
| Drive controls | **NOT STARTED** | No joystick-to-drivetrain bindings exist yet |
| Intake button bindings | **NOT STARTED** | Intake has no controller buttons assigned |
| Climb button bindings | **NOT STARTED** | Climb has no controller buttons assigned |
| `shuttleFuel()` binding | **NOT STARTED** | Function exists but no button triggers it |
| `shootFuel()` binding | **NOT STARTED** | Command exists but is never scheduled |

---

## 16. Troubleshooting — When Things Go Wrong

### "I cannot build the project"

1. Make sure you installed **WPILib 2026** (not an older version)
2. Try `./gradlew clean` then `./gradlew build`
3. Make sure you have internet (it needs to download libraries the first time)
4. If you see "Java version" errors, WPILib comes with its own Java — make sure your terminal is using it

### "Deploy fails / cannot connect to robot"

1. Check your connection:
   - **USB:** Plug a USB-B cable from your laptop directly to the RoboRIO
   - **Wi-Fi:** Connect to the robot's radio network (usually named something like "6059_RADIO")
2. Try pinging the robot: `ping 10.60.59.2`
3. Make sure the team number is correct in `.wpilib/wpilib_preferences.json` (should be `6059`)
4. Try restarting the RoboRIO (there is a small reset button on it)

### "Motor not found on CAN bus"

This means the code is trying to talk to a motor, but cannot find it:

1. **Check wiring** — The yellow and green CAN wires must be connected in a chain from the RoboRIO through each motor/sensor
2. **Check the CAN ID** — Open **Phoenix Tuner X**, connect to the robot, and scan for devices. Make sure the motor's ID matches what is in `Constants.java`
3. **Check for duplicate IDs** — Two devices with the same ID will cause problems
4. **Update firmware** — Phoenix Tuner X can update motor firmware. Old firmware can cause communication issues

### "Shooter flywheel is not reaching target speed"

1. **Check battery voltage** — A low battery (below 11V) cannot provide enough power. Charge or replace the battery.
2. **Check for jams** — Make sure nothing is stuck in the flywheel mechanism
3. **Look at telemetry** — Open AdvantageScope and check `Shooter/FlywheelVelocity` vs `Shooter/FlywheelTargetVelocity`. If the actual velocity is climbing slowly, the PID might need more kP or kV.

### "Hood is oscillating (vibrating back and forth)"

This usually means the PID is overcorrecting:
1. **Reduce kP** — In `Constants.java`, lower the `HOOD_KP` value (try halving it)
2. **Increase kD** — This adds dampening. Try doubling `HOOD_KD`
3. **Check the CANcoder magnet** — If the magnet is loose or misaligned, the sensor readings will be noisy

### "Error: Subsystem already in use / Command scheduling conflict"

This means two commands are trying to control the same mechanism at the same time:
1. Check that button bindings use `.onlyIf(() -> mode == Mode.MANUAL)` guards where appropriate
2. Make sure commands properly finish (they should stop when the button is released)
3. Only one command can use a subsystem at a time — this is by design

### Useful Debugging Tools

| Tool | What It Does | When to Use It |
|---|---|---|
| **FRC Driver Station** | Shows robot status, lets you enable/disable, displays console output | Always — this is your main dashboard |
| **Phoenix Tuner X** | Scan for CAN devices, test motors manually, update firmware, plot signals in real time | When motors are not working, or when setting up new hardware |
| **AdvantageScope** | Visualize logged data (graphs, 3D views, tables). Works with DogLog data. | When you need to see what happened during a test run |
| **WPILib Simulation GUI** | Simulates the robot on your laptop — fake motors, fake sensors, fake controller | When you do not have access to the robot and want to test logic |
| **Shuffleboard** | A dashboard for displaying and editing values while the robot runs | When you want live feedback during testing |

---

## 17. Complete Glossary — Every Term Explained

If you see a word you do not understand anywhere in this document or in the code, check here.

| Term | What It Means |
|---|---|
| **Absolute encoder** | A sensor that knows its exact position even after the robot is restarted. Like a clock — it always knows the time, not just how much time has passed. Our CANcoders are absolute encoders. |
| **AdvantageScope** | A free application for viewing robot data logs. You record data on the robot, download it, and replay it in AdvantageScope to see what happened. |
| **Autonomous (auto)** | The first ~15 seconds of a match where the robot must operate without any human control. The robot follows pre-programmed instructions. |
| **Brake mode** | A motor setting. When the motor is told to stop, it actively holds its position instead of freewheeling. Like putting a car in park vs. neutral. |
| **Brownout** | When too many motors draw power at once and the battery voltage drops too low. The RoboRIO may disable motors to protect itself. Avoid by limiting peak current. |
| **CAN bus** | Controller Area Network. A communication system where all motors and sensors are daisy-chained together on two wires (yellow and green). Each device has a unique ID number. |
| **CAN ID** | A unique number (0-62) assigned to each device on the CAN bus. Like a street address — the code uses this number to send instructions to the right motor/sensor. |
| **CANcoder** | An absolute position sensor made by CTRE. Uses a magnet to detect rotation. Knows its exact angle even after power off/on. |
| **CTRE** | Cross The Road Electronics. The company that makes our TalonFX motors and CANcoder sensors. |
| **Coast mode** | A motor setting. When the motor is told to stop, it freewheels to a halt naturally. Like a bike when you stop pedaling. |
| **Command** | An action the robot performs using one or more subsystems. Examples: "spin the flywheel," "extend the climb arm." Commands are created and bound to buttons. |
| **Command scheduler** | The "brain" that runs all active commands every 20 ms. It makes sure no two commands use the same subsystem at once. Provided by WPILib — you do not write it. |
| **Closed-loop control** | The motor actively adjusts itself to reach a target (speed or position) using sensor feedback and PID math. "Smart" control. See Section 6.7. |
| **Constants** | Numbers that do not change while the code runs (motor IDs, speeds, PID gains). All stored in `Constants.java` for easy editing. |
| **Current limit** | A safety setting that prevents a motor from drawing too much electrical current. Too much current can damage motors, wires, or breakers. Measured in amps. |
| **Deploy** | Sending compiled code from your laptop to the RoboRIO. Done with `./gradlew deploy`. |
| **Driver Station** | Software on the driver's laptop that communicates with the robot. Used to enable/disable the robot and view status. Required for all robot operation. |
| **DogLog** | A simple logging library for FRC. Records values (like motor speeds) so you can review them later. |
| **DutyCycleOut** | A way to control a motor by setting a percentage of full power. 0.5 = 50% power. 1.0 = full power. -1.0 = full power in reverse. This is open-loop control. |
| **Encoder** | A sensor that measures rotation. Built-in encoders count rotations from startup. Absolute encoders (like CANcoder) know their exact position always. |
| **Feedforward (kV)** | A prediction: "To reach speed X, I probably need Y voltage." Gives the PID a head start so it only needs to make small corrections. |
| **Flywheel** | A spinning wheel used to launch game pieces. In our shooter, two flywheels spin in opposite directions to grip and fling the game piece. |
| **FRC** | FIRST Robotics Competition. A worldwide high school robotics competition. |
| **FusedCANcoder** | A Phoenix 6 feature that combines the absolute position of a CANcoder with the fast updates of the motor's built-in encoder. Best of both worlds. |
| **Gain** | A number that controls how strongly a PID term reacts. Higher gain = stronger reaction. Too high = oscillation. Too low = slow response. |
| **Gear ratio** | How many times the motor shaft spins for one rotation of the mechanism. A 50:1 ratio means the motor spins 50 times for the mechanism to spin once. Higher ratios give more torque (strength) but slower speed. |
| **Gradle** | A build tool that compiles Java code, downloads libraries, and packages everything for deployment. You interact with it through `./gradlew` commands. |
| **GradleRIO** | A Gradle plugin specifically for FRC. Adds commands like `deploy` and `simulateJava`. |
| **Hood** | An adjustable ramp on top of the shooter that controls the launch angle. Tilting it changes whether the game piece goes high or low. |
| **Java** | The programming language we use. Java 17 is the version. |
| **Linear interpolation** | Estimating a value between two known points by drawing a straight line between them. Used in lookup tables. |
| **Match** | An FRC game. Usually ~2.5 minutes with autonomous (robot drives itself) and teleop (human drives) phases. |
| **Neutral mode** | What a motor does when it is told to stop. Either **brake** (hold position) or **coast** (freewheel). |
| **Open-loop control** | Telling a motor to run at a set power percentage without measuring the result. Simple but imprecise. See Section 6.7. |
| **Oscillation** | When a mechanism vibrates back and forth instead of holding steady. Usually caused by PID gains being too high. |
| **PathPlanner** | A tool for creating autonomous driving paths. You draw a path on a field image, and the robot follows it. |
| **Periodic** | Something that happens at regular intervals. `robotPeriodic()` runs every 20 ms (50 times per second). |
| **Phoenix 6** | The software library from CTRE for controlling TalonFX motors and CANcoders. Version 6 is the latest. |
| **Phoenix Tuner X** | A desktop application for configuring CTRE hardware. Use it to set CAN IDs, test motors, update firmware, and view real-time data. |
| **PID** | Proportional-Integral-Derivative. A math-based control method that automatically adjusts motor power to reach and hold a target. See Section 6.3 for full explanation. |
| **PositionVoltage** | A control mode that uses PID to move a motor to a specific position (angle or number of rotations) and hold it there. |
| **Regression** | A mathematical technique for finding a formula (curve) that best fits a set of data points. We use it to calculate shooter speeds and angles from distance. |
| **RoboRIO** | The robot's main computer. Made by National Instruments. Runs our Java code. Connects to motors, sensors, and the Driver Station. Powered by the robot battery. |
| **Subsystem** | A Java class representing one physical mechanism on the robot. Each subsystem owns its motors/sensors and ensures only one command controls it at a time. See Section 6.1. |
| **Swerve drive** | A drivetrain where each wheel can spin AND steer independently. This lets the robot drive in any direction without turning. Our robot has a swerve config file but the code is not written yet. |
| **TalonFX** | A motor controller made by CTRE, built into Falcon 500 and Kraken X60 motors. Has a built-in encoder, communicates over CAN bus, and supports advanced control modes. |
| **Teleop (teleoperated)** | The ~2 minute phase of a match where a human driver controls the robot using a controller. |
| **Telemetry** | Data recorded from the robot during operation (motor speeds, sensor readings, etc.). Used for debugging and analysis after a match. |
| **TimedRobot** | A WPILib base class that runs code in a loop at 50 Hz (every 20 ms). Our `Robot.java` extends this. |
| **Tolerance** | The "close enough" threshold. If the target is 50 ft/s and the tolerance is 2 ft/s, then any speed between 48 and 52 ft/s counts as "at target." |
| **Tuning** | The process of finding the right PID gain values by testing on the real robot. You adjust kP, kI, kD until the mechanism moves smoothly and accurately. |
| **USB** | Universal Serial Bus. A cable connection. Used to connect the Xbox controller to the laptop, and optionally to connect the laptop directly to the RoboRIO for deploying code. |
| **Vendor dependency** | A third-party library (code someone else wrote) that we use in our project. Defined by JSON files in the `vendordeps/` folder. |
| **VelocityVoltage** | A control mode that uses PID to spin a motor at a specific speed and hold it there. |
| **VS Code** | Visual Studio Code. The code editor we use. WPILib installs a special version with FRC tools built in. |
| **WPILib** | The official FRC software library. Provides tools for controlling motors, sensors, controllers, and more. Also includes build tools, simulation, and the command-based framework. |

---

*This documentation was written for FRC Team 6059 — Robot2026. If any term is unclear or missing, ask a mentor or add it to the glossary.*
