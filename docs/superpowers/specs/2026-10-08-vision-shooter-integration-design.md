# Vision + Shooter Integration Design (Robot2026 / Arc-imedes)

**Date:** 2026-10-08
**Status:** Draft for review
**Branch:** `20261005-vedha-vision-multitags`

## 1. Goal

Give the driver two buttons on Arc-imedes:

1. **Align to hub:** turn the robot so the shooter points at its alliance's hub, using the
   2026 field layout and two cameras. The driver can still drive (translate) while holding it.
2. **Align and shoot:** align, set the hood and flywheel for the distance, and feed fuel once
   everything is on target.

Keep the existing manual and calibration shooter buttons working, and document every button
when the work is done.

## 2. What we know about the robot

These were confirmed with the team on 2026-10-08.

- **Robot frame (WPILib):** +X is the front (RSL side, drive-forward), +Y is left (intake side),
  +Z is up.
- **RSL and front camera:** front-right corner. The front camera sits under the front end of the
  hood and faces forward.
- **Back camera:** back-right corner, facing backward.
- **Shooter:** a single hood pivots over the flywheel to either side, along the right side of
  the robot.
- **Front shot:** flywheel CCW. Fuel leaves the back-right end of the hood and flies toward the
  front. The front of the robot faces the hub and the front camera sees it.
- **Back shot:** flywheel CW. Fuel leaves the front-right end of the hood and flies toward the
  back. The back of the robot faces the hub and the back camera sees it.
- **Hood angle scale:** signed, with 0° meaning the hood is centered over the flywheel. The
  motor soft limits are ±90°. The existing presets are −88° (front shot, +42 ft/s) and +87°
  (back shot, −42 ft/s).

## 3. Assessment of existing code

### 3.1 Problems in Robot2026's current vision code (`PointToHub`)

| # | Problem | Where | Effect |
|---|---|---|---|
| 1 | Vision never updates the robot's pose | `Robot.robotPeriodic()`: `pointToHub.update()` is commented out | The robot runs on wheel odometry only; the cameras never correct drift |
| 2 | Would crash if turned on | `PointToHub.updatePose()` calls `visionEst.get()` without checking it's present | The first frame without a usable pose throws an exception |
| 3 | Camera timestamps converted twice | `updatePose()` calls `Utils.fpgaToCurrentTime()`, then `CommandSwerveDrivetrain.addVisionMeasurement()` converts again | Each measurement would be applied at the wrong moment |
| 4 | Aim math mixes two frames | `targetYaw` is the hub angle relative to the shooter, but it's compared with the robot's field heading | The controller settles part-way to the target instead of on it, unless the robot already faces 0° |
| 5 | No check on bad measurements | Every pose gets fixed trust (`DEFAULT_VISIONSTDEV`) | Ambiguous or far-away single-tag readings are trusted as much as good ones |
| 6 | Hub location is wrong | One tag (26 blue, 10 red) plus a ±1 ft offset | The hub center is about 0.6 m behind the tag face. The offset moves the aim point 1–3 ft away from the real center. |
| 7 | Alliance read once, at boot | `PointToHub` constructor | If the Driver Station hasn't reported an alliance yet, no hub is chosen |
| 8 | Camera positions don't match the robot | `Constants.Vision` puts the cameras on the left and right, facing sideways | Every vision pose and distance would be wrong |
| 9 | Silent camera-name mismatch | `new PhotonCamera(name)` | A typo in a camera name looks exactly like "no tags in view" |

### 3.2 What 2026VisionBotoffSeason does better

| Area | 2026VisionBotoffSeason | Robot2026 now |
|---|---|---|
| Pose fusion | `VisionSubsystem.periodic()` feeds every frame from every camera into the drivetrain's pose estimator, all the time | Never runs |
| Filtering | `VisionFilter` rejects ambiguous single-tag solves, poses off the floor, far single tags, poses outside the field and sudden jumps. Trust falls with distance². | None |
| Aiming | Aims using the fused pose and CTRE's built-in heading controller (`FieldCentricFacingAngle`), so losing a tag doesn't make the target jump | Steers on raw camera readings, with the frame bug above |
| Finishing | Settles for several loops before finishing, has a no-progress watchdog and a 15 s timeout | Runs while held |
| Safety | Refuses to start if no camera has confirmed the pose in the last second | None |
| Camera health | Loud error if a camera name doesn't match PhotonVision | Silent |
| Tests | Unit tests for the geometry and the filter | None |
| Generated code | Additions live in a subclass (`VisionSwerveDrivetrain`), safe from Phoenix Tuner X regeneration | Edits generated code |

### 3.3 What the port lacks for Arc-imedes

- Always aims the robot's front; Arc-imedes must aim whichever end is better.
- Aims at a tag, not the hub center.
- Aims from the robot center, not the shooter's launch point.
- No shooter integration.
- Defaults to the practice layout (`kUsePracticeLayout = true`).
- Turn rate capped at 25°/s (a 180° turn takes about 7 s).
- Camera positions are centerline placeholders.
- Writes a CSV file on every command run.
- Never run on a swerve robot; all gains are untuned.

**Decision:** port the 2026VisionBotoffSeason architecture into Robot2026 and adapt it for
Arc-imedes. Delete `PointToHub`.

## 4. Design

### 4.1 Components

| Unit | Kind | Job | Depends on |
|---|---|---|---|
| `VisionSwerveDrivetrain` | Subsystem (subclass of `CommandSwerveDrivetrain`) | Fused pose, yaw rate, `driveFacingAngle(vx, vy, heading)` | Generated swerve code |
| `VisionSubsystem` | Subsystem | Reads both cameras, filters solves, feeds the pose estimator, reports camera health and time since last accepted measurement, and resets the pose translation from vision on request (Start button) | PhotonLib, `VisionFilter`, field layout |
| `VisionFilter` | Pure class | Accept or reject a solve, and pick its trust (std devs) | Constants only |
| `FieldTargets` | Pure class | Hub center for an alliance, from the layout's hub tags | `AprilTagFieldLayout` |
| `ShotDirection` | Enum | `FRONT` / `BACK`: which robot end faces the hub, flywheel sign, hood sign, camera | Constants |
| `TargetSolution` | Pure record | Distance and bearing from the launch point to the hub, and the heading that puts the shot on target | WPILib geometry |
| `ShotTable` | Pure class | Distance → hood angle and flywheel speed, one table per `ShotDirection` | WPILib `InterpolatingDoubleTreeMap` |
| `AlignToHubCommand` | Command (needs drivetrain) | Turns to the aim heading while the driver translates | All of the above |
| `AlignAndShootCommand` | Command (needs drivetrain, shooter, hopper) | Aligns, spins up, sets the hood, feeds when ready | All of the above |
| `ShootAtDistanceCommand` | Command (needs shooter, hopper) | Manual mode: shoot using the driver-set distance | `ShotTable` |
| `CalibrationShootCommand` | Command (needs shooter, hopper) | Calibration mode: shoot at the hand-set hood angle and flywheel speed | Shooter |
| `CalibrationState` | Plain class | Holds the calibration direction, hood angle, flywheel speed and distance source; syncs with dashboard entries | `ShotTable`, NetworkTables |
| `ShotRecorder` | Plain class | Saves a calibration shot as a row: log, CSV file, dashboard, paste-ready table line | DogLog, file system |

Files ported from 2026VisionBotoffSeason with their tests: `VisionSubsystem`, `VisionFilter`,
`TargetSolution`, `VisionSwerveDrivetrain`, `SettleDetector`, `TagFieldLayoutLoader`.
(`ProgressWatchdog` is not needed: every command is hold-to-run or has a timeout.) **Not** ported: `CsvLogger` (DogLog covers it), `MapVisibleTagsCommand`,
`DriveToTagCommand`, `DriveRelativeCommand`, `DriveRoutines`.

**Deleted:** `PointToHub`, `ShooterCalculator` (its angle function returns a constant 2°),
the unused left/right shooter transforms in `Constants.Vision`.

### 4.2 Cameras

Two `CameraConfig` entries in `Constants.Vision`:

| Camera | Position (robot frame) | Yaw | Pitch |
|---|---|---|---|
| Front | front-right corner, under the hood: X > 0, Y < 0 | 0° | negative if tilted up |
| Back | back-right corner: X < 0, Y < 0 | 180° | negative if tilted up |

The numbers are placeholders until measured in the lab (see section 7). PhotonVision camera
names also go in these entries and must match the web UI exactly.

### 4.3 Field layout and hub center

- Official layout: `AprilTagFields.k2026RebuiltWelded` by default. A constant switches it to
  `k2026RebuiltAndymark` for events that use AndyMark fields.
- Hub tags (from the 2026 layout, height 1.124 m):
  blue hub = 18, 19, 20, 21, 24, 25, 26, 27; red hub = 2, 3, 4, 5, 8, 9, 10, 11.
- `FieldTargets.hubCenter(alliance)` returns the center of that hub's tag positions'
  bounding box, so it matches the field file. For the welded field this is
  (4.626, 4.034) m for blue and (11.916, 4.034) m for red.
- The alliance is read when a command starts. If it's unknown, the command doesn't move and
  reports "no alliance" on the dashboard.
- **Lab mode:** the practice-layout loader is kept, set off by default. With the practice
  layout on, the commands aim at a configured lab tag (`kLabTargetTagId`) instead of the hub,
  so the team can test with a single tag on a wall.

### 4.4 Choosing the shot direction

When an align command starts:

1. Work out the aim heading for a front shot and for a back shot (section 4.5).
2. Compare each with the robot's current heading, measuring the back shot from the robot's
   back end (heading + 180°).
3. Pick the direction with the smaller turn.
4. Keep that direction until the button is released, so it can't flip mid-turn.

The direction decides:

| | FRONT | BACK |
|---|---|---|
| Robot end facing the hub | front (heading = aim) | back (heading = aim + 180°) |
| Flywheel sign | + (CCW, Phoenix default) | − (CW) |
| Hood sign | − | + |
| Camera that should see the hub | front | back |

The flywheel and hood signs come from today's trench presets and are checked in the lab
before use (TODO item).

### 4.5 Aim heading with the shooter offset

The fuel launches from a point on the right side of the robot, not from the center.
`TargetSolution` aims that launch point at the hub center:

- `P` = robot position (fused pose), `H` = hub center, `β` = bearing from `P` to `H`,
  `D` = distance from `P` to `H`.
- `e` = how far the launch point sits to the left of the firing line through the robot center.
  For a front shot `e = launchY`; for a back shot `e = −launchY` (`launchY` is negative, since
  the hood is on the right).
- Firing direction: `θ = β − asin(e / D)`. The robot heading is `θ` for a front shot and
  `θ + 180°` for a back shot.
- Shot distance (used for the shot table) is the horizontal distance from the launch point
  to `H`.

`launchX`/`launchY` start at 0 (no correction) until measured. With `e = 0` this is the same
as aiming the robot center.

### 4.6 Shot table

- One table per direction, mapping distance (ft) to hood angle (° on the signed scale) and
  flywheel speed (ft/s), with linear interpolation between points and clamping at the ends.
- Stored as positive numbers. `ShotDirection` applies the signs.
- Starting data: one point per direction, taken from the trench presets (88°, 42 ft/s). With one
  point the table returns that value at every distance, which matches today's behavior.
  More points get added in the lab.
- Each table is clamped to that direction's usable hood range, measured in the lab.

### 4.7 Align to hub (hold X)

- Uses `driveFacingAngle(driverVx, driverVy, aimHeading)`, so the driver keeps full control of
  translation with the left stick.
- Recomputes the aim heading every loop from the fused pose.
- Runs until released. It never refuses to run; it shows `Aim/VisionFresh` on the dashboard so
  the driver knows whether vision has confirmed the pose recently.
- Publishes `Aim/Direction`, `Aim/HeadingErrorDeg`, `Aim/DistanceFt`, `Aim/Aimed`.

### 4.8 Align and shoot (hold B)

While held:

1. Align exactly as in 4.7.
2. Every loop, look up the hood angle and flywheel speed for the current distance and send them
   to the shooter.
3. Feed (hopper `FORWARD`) only when **all** of these are true, otherwise hopper `OFF`:
   - heading error ≤ `kAimToleranceDeg` (start at 2°), held for `kSettledLoops` loops
   - yaw rate < `kSettledRateDegPerSec`
   - robot speed < `kMaxShootSpeedMps` (start at 0.3 m/s; no shooting on the move)
   - `shooter.isAtTarget()`
   - vision accepted a measurement within the last `kMaxVisionAgeToFeedSec` (start at 1 s)
4. Once feeding starts, keep feeding while the conditions hold. If they drop out, stop feeding
   and wait for them again.

On release: flywheel 0, hood idle, hopper off. `Shooter/ReadyToFire` shows the feed decision,
and `Shooter/BlockedBy` names the first unmet condition.

The feed decision is a pure function (`ShotGate.canFeed(...)`) so it can be unit-tested.

### 4.9 Manual and calibration shooting

Today's periodic `updateShooter()` and `updateShooterInCalibration()` are commented out. If
they ran, they would also overwrite the D-pad presets every loop. They are replaced by commands
that only run while the trigger is held:

- **MANUAL mode, hold right trigger:** `ShootAtDistanceCommand` uses the driver-set distance
  (bumpers, ±1 ft) and the selected `ShotDirection` (A toggles it), looks up the shot table,
  and feeds when `shooter.isAtTarget()`.
- **CALIBRATION mode, hold right trigger:** `CalibrationShootCommand` shoots at the calibration
  hood angle and flywheel speed and feeds when the shooter is at target (section 4.10).
- **D-pad left / right outside calibration:** front / back trench presets, unchanged.

The AUTO shooter mode is removed: aiming now lives on X and B in every mode. Y no longer
toggles AUTO; it's left free. **Decided 2026-10-08 (option A).** The alternative, keeping a Y
toggle so the right trigger can shoot with the vision distance while the driver aims by hand
(option C), is not needed for now; a TODO item asks to confirm with the driver.

### 4.10 Calibration mode for building the shot table

Purpose: let the team find, at each distance and in each direction, the hood angle and flywheel
speed that score, and save them as shot-table rows without hand-copying numbers.

**Controls (CALIBRATION mode, D-pad up to enter or leave):**

| Input | Action |
|---|---|
| A | Switch calibration direction FRONT / BACK. Hood and flywheel signs flip with it, and the values reset to that direction's table entry for the current distance. |
| D-pad left / right | Hood −2.5° / +2.5° (magnitude; the direction supplies the sign) |
| Bumpers | Flywheel −2 / +2 ft/s (magnitude) |
| Right trigger (hold) | Spin up to the calibration values; feed automatically once `shooter.isAtTarget()`; stop and idle on release |
| Y | Save the current shot as a table row |

**Starting values:** on entering CALIBRATION, and on every A press, the hood and flywheel start
from the shot table's entry for the current distance and direction, not from 0° / 50 ft/s.

**Typed values (dashboard):** `CalibrationState` publishes and reads these NetworkTables entries
(editable in Elastic, Shuffleboard or AdvantageScope):

| Entry | Meaning |
|---|---|
| `Cal/Direction` | FRONT or BACK (read-only display) |
| `Cal/HoodDeg` | Hood angle magnitude (editable) |
| `Cal/FlywheelFps` | Flywheel speed magnitude (editable) |
| `Cal/UseManualDistance` | true = use `Cal/ManualDistanceFt` instead of vision (editable) |
| `Cal/ManualDistanceFt` | Tape-measured distance from the launch point to the hub (editable) |
| `Cal/DistanceFt` | Distance used for the row (read-only: vision or manual) |
| `Cal/DistanceSource` | VISION or MANUAL (read-only) |
| `Cal/VisionFresh` | true if vision confirmed the pose within 1 s (read-only) |
| `Cal/ActualFlywheelFps`, `Cal/ActualHoodDeg` | Measured values (read-only) |
| `Cal/LastSaved` | The last saved row as text (read-only) |

Button presses and typed values change the same numbers, so either works. Typed values are
clamped to the direction's usable hood range and to ±90 ft/s.

**Distance:** vision distance from the launch point to the hub center (to the lab tag in lab
mode). If vision isn't fresh and `Cal/UseManualDistance` is false, Y refuses to save and
`Cal/LastSaved` says why.

**Saving a shot (Y):** `ShotRecorder` writes one row for a shot that scored:

- Fields: time, direction, distance (ft), distance source, hood set (°), hood actual (°),
  flywheel set (ft/s), flywheel actual (ft/s), vision fresh (true/false).
- Written to DogLog (`Cal/Saved/...`), appended to `/home/lvuser/logs/shot-calibration.csv`, and
  shown in `Cal/LastSaved`.
- Also printed to the console as a paste-ready line, for example
  `{9.0, 64.5, 38.0}, // BACK: distance ft, hood deg, flywheel ft/s`, to paste into
  `ShotConstants.kFrontShots` or `kBackShots`.
- Only scoring shots are saved; misses are just not recorded.

**Workflow:** place the robot at a marked distance (for example 6, 9, 12, 15 ft), pick the
direction with A, adjust until shots score consistently, press Y, repeat for each distance and
both directions, then paste the rows into `ShotConstants.kFrontShots` / `kBackShots`.

### 4.11 Autonomous

Add PathPlanner named command `alignAndShoot`: runs `AlignAndShootCommand` until it has fed for
`kAutoFeedSeconds` (start at 1.5 s), with a 4 s overall timeout. Existing named commands are
unchanged.

### 4.12 Error handling

| Situation | Behavior |
|---|---|
| A camera name doesn't match PhotonVision | Loud `DriverStation` error 5 s after enable, naming the camera (from the port) |
| Both cameras down | Aiming still works on odometry; feeding is blocked by the vision-age check |
| Alliance unknown | Align commands don't move; `Aim/Direction` shows `NO_ALLIANCE` |
| Practice layout on, but the file failed to load | Pose fusion and targeting disabled (from the port) |
| Hub out of shot-table range | The table clamps to its nearest entry; `Shooter/OutOfRange` is true |
| Y pressed with no trusted distance | No row saved; `Cal/LastSaved` explains (no fresh vision, manual distance off) |
| CSV file can't be written | Row still goes to DogLog and the console; one warning printed |

## 5. Buttons after this change (Xbox, port 0)

| Input | Any mode | MANUAL | CALIBRATION |
|---|---|---|---|
| Left stick / right stick X | Drive / rotate | | |
| **X (hold)** | **Align to hub** | | |
| **B (hold)** | **Align and shoot** | | |
| Right trigger (hold) | | Shoot at set distance | Shoot at calibration values, auto-feed when on target |
| A | | Toggle shot direction FRONT / BACK (X and B pick the direction themselves) | Toggle calibration direction; reset values to the table entry |
| Bumpers | | Distance −1 / +1 ft | Flywheel −2 / +2 ft/s |
| D-pad left / right | | Front / back trench preset | Hood −2.5° / +2.5° |
| D-pad up | Toggle CALIBRATION | | |
| D-pad down | unchanged | | |
| Left trigger | Toggle intake | | |
| Start (release) | Reset pose translation from vision | | |
| Back | Unchanged (brake / invert; known double binding, tracked separately) | | |
| Y | | Free | Save the current shot as a table row |

## 6. Testing

- **Unit tests** (run with `./gradlew test`):
  - Ported tests for `VisionFilter`, `TargetSolution`, `SettleDetector`, `ProgressWatchdog`,
    `TagFieldLayoutLoader`.
  - `FieldTargets`: hub centers for both alliances and both layouts match the numbers above.
  - Direction choice: robot facing the hub picks FRONT; facing away picks BACK; ties at 90°
    pick FRONT.
  - Aim heading: zero offset gives the plain bearing; a right-side launch point turns the aim
    the right way for both directions; wraparound at ±180°.
  - `ShotTable`: interpolation, clamping, signs per direction, single-point behavior.
  - `ShotGate.canFeed`: each condition blocks feeding on its own.
  - `CalibrationState`: direction switch flips signs and resets to the table entry; typed values
    are clamped; button steps and typed values stay in sync.
  - `ShotRecorder`: row formatting, CSV header and paste-ready line; refuses without a trusted
    distance.
- **Simulation:** `./gradlew simulateJava` to check the buttons bind and commands run without
  exceptions (no camera simulation).
- **Lab bring-up** (in order): camera names and health, camera positions against a tape
  measure, hood zero and signs, flywheel direction, align with one lab tag, align and shoot at
  one distance, then fill in the shot table.

## 7. Lab measurements needed

These are placeholders in code until measured:

1. Hood zero, sign per direction, end stops and usable range (TODO item with steps).
2. Front and back camera position and tilt (X, Y, Z, pitch).
3. PhotonVision camera names.
4. Launch point `launchX`, `launchY` (where the fuel leaves the hood, for each direction).
5. Flywheel direction check (front shot = positive speed?).
6. Shot table points at several distances for each direction.
7. Heading controller gain, aim tolerance and max turn rate on the real swerve.
8. Whether events use the welded or AndyMark field.

## 8. Out of scope

- Shooting while moving (no velocity compensation).
- Changes to existing autos (only a new named command is added).
- Climb, the Back-button double binding, D-pad down heading check, and the other merge
  follow-ups (tracked in `TODO.md`).
- Tag mapping tools from the practice-bot repo.

## 9. Documentation to update when done

- `docs/CODEMAP.md`: new vision classes, commands and the button table.
- `docs/shooter-vision-guide.html` (and its published page): align/shoot buttons, dashboard
  values and new troubleshooting rows.
