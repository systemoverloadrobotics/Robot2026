# Vision + Shooter Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two-camera vision pose fusion, hold-to-run "align to hub" (X) and "align and shoot" (B), a distance-based shot table, and a calibration mode that records shot-table rows, to Robot2026 (Arc-imedes).

**Architecture:** Port the 2026VisionBotoffSeason vision stack (`VisionSubsystem`, `VisionFilter`, `TargetSolution`, `VisionSwerveDrivetrain`, `SettleDetector`, `TagFieldLayoutLoader`), then add pure, unit-tested targeting classes (`ShotDirection`, `FieldTargets`, `HubAim`, `ShotTable`, `ShotGate`, `CalibrationState`, `ShotRecorder`) and thin commands that wire them to the drivetrain, shooter and hopper. `PointToHub`, `ShooterCalculator` and the AUTO shooter mode are removed.

**Tech Stack:** Java 17, WPILib 2026.2.1 (GradleRIO), CTRE Phoenix 6 swerve, PhotonLib, PathPlannerLib, DogLog, JUnit 5.

**Spec:** `docs/superpowers/specs/2026-10-08-vision-shooter-integration-design.md`

## Global Constraints

- Robot frame: +X front (RSL side), +Y left (intake side), +Z up. Angles CCW-positive.
- Hood angles use the signed scale: 0° = hood centered over the flywheel, soft limits ±90°.
- FRONT shot: flywheel sign +, hood sign −, robot front faces the hub. BACK shot: flywheel −, hood +, robot back faces the hub.
- Shot table units: distance ft, hood ° (magnitude), flywheel ft/s (magnitude). `ShotDirection` applies signs.
- Hub tags: blue 18, 19, 20, 21, 24, 25, 26, 27; red 2, 3, 4, 5, 8, 9, 10, 11. Hub center = bounding-box center of those tag positions.
- Field layout: `k2026RebuiltWelded` by default; `FieldConstants.kUseAndyMarkField` switches.
- Camera positions, camera names, launch points and hood ranges are PLACEHOLDERS until the lab steps; mark them so in code.
- Do not edit `generated/TunerConstants.java` or `subsystems/CommandSwerveDrivetrain.java` (Tuner X generated); extend by subclass.
- Do not commit unless the user asks (the branch holds other uncommitted work).
- Build check after every task: `./gradlew build --offline` (compiles and runs unit tests).

## Review Focus

1. **Red alliance driving while aiming:** driver stick input is operator-relative, the aim request is blue-origin field-relative. Expect the left stick to move the robot the same way during X/B as during normal driving on red. Pinned by `HubAimTest.operatorToFieldFlipsOnRed` (Task 3).
2. **Robot almost on top of the target:** distance ≈ 0 or smaller than the launch-point offset must not produce NaN headings. Pinned by `HubAimTest.launchOffsetLargerThanDistanceDoesNotProduceNaN` (Task 3).
3. **Shot table lookups outside the measured distances:** expect clamping to the nearest row, never 0 or an exception. Pinned by `ShotTableTest.clampsBeyondEnds` (Task 4).
4. **Calibration typed values outside the hood range or negative flywheel:** expect clamping, not motor commands past the soft limits. Pinned by `CalibrationStateTest.typedValuesAreClamped` (Task 8).
5. **Saving a calibration row with no trusted distance:** expect a refusal message, no row. Pinned by `ShotRecorderTest.refusesWithoutTrustedDistance` (Task 8).

---

## File Structure

| File | Status | Responsibility |
|---|---|---|
| `src/main/java/frc/robot/Constants.java` | Modify | Remove `Vision`; add `FieldConstants`, `VisionConstants`, `AimConstants`, `ShotConstants` |
| `src/main/java/frc/robot/util/SettleDetector.java` | Create (port) | N consecutive "aimed and stopped" loops |
| `src/main/java/frc/robot/targeting/VisionFilter.java` | Create (port) | Accept/reject a vision pose, choose std devs |
| `src/main/java/frc/robot/targeting/TagFieldLayoutLoader.java` | Create (port + AndyMark switch) | Load official or practice layout |
| `src/main/java/frc/robot/targeting/TargetSolution.java` | Create (port) | Distance/bearing robot → target |
| `src/main/java/frc/robot/targeting/ShotDirection.java` | Create | FRONT/BACK signs, facing offset, launch point |
| `src/main/java/frc/robot/targeting/ShotSetpoint.java` | Create | Signed hood ° + flywheel ft/s |
| `src/main/java/frc/robot/targeting/FieldTargets.java` | Create | Hub center from layout |
| `src/main/java/frc/robot/targeting/HubAim.java` | Create | Aim heading with launch offset, direction choice, operator→field |
| `src/main/java/frc/robot/targeting/ShotTable.java` | Create | Distance → hood/flywheel interpolation + clamping |
| `src/main/java/frc/robot/targeting/ShotGate.java` | Create | Feed decision |
| `src/main/java/frc/robot/subsystems/VisionSwerveDrivetrain.java` | Create (port) | Aim request, fused pose, yaw rate |
| `src/main/java/frc/robot/subsystems/VisionSubsystem.java` | Create (port + additions) | Cameras → pose estimator; aim target; reset from vision |
| `src/main/java/frc/robot/commands/AlignToHubCommand.java` | Create | Hold-X aim |
| `src/main/java/frc/robot/commands/AlignAndShootCommand.java` | Create | Hold-B aim + shoot |
| `src/main/java/frc/robot/commands/ShootCommand.java` | Create | Spin to a setpoint, feed when ready (manual + calibration) |
| `src/main/java/frc/robot/calibration/CalibrationState.java` | Create | Calibration direction/values with clamping |
| `src/main/java/frc/robot/calibration/CalibrationDashboard.java` | Create | `Cal/*` NetworkTables sync |
| `src/main/java/frc/robot/calibration/ShotRecorder.java` | Create | Save rows: DogLog, CSV, paste line |
| `src/main/java/frc/robot/RobotContainer.java` | Modify | New subsystems, bindings, remove AUTO mode |
| `src/main/java/frc/robot/Robot.java` | Modify | Call `m_robotContainer.periodic()` |
| `src/main/java/frc/robot/commands/PointToHub.java` | Delete | Replaced |
| `src/main/java/frc/robot/utils/ShooterCalculator.java` | Delete | Replaced by `ShotTable` |
| `src/test/java/frc/robot/**` | Create | Unit tests |
| `docs/CODEMAP.md`, `docs/shooter-vision-guide.html`, `TODO.md` | Modify | Document buttons and lab steps |

Note vs spec: `ProgressWatchdog` is not ported (all commands are hold-to-run or have a timeout). Manual and calibration shooting share one `ShootCommand` with a setpoint supplier instead of two classes. Paste lines go into `ShotConstants.kFrontShots` / `kBackShots`.

---

### Task 1: Constants, ported utilities and their tests

**Files:**
- Modify: `src/main/java/frc/robot/Constants.java` (replace `public static class Vision {…}`)
- Create: `src/main/java/frc/robot/util/SettleDetector.java`, `src/main/java/frc/robot/targeting/VisionFilter.java`, `src/main/java/frc/robot/targeting/TagFieldLayoutLoader.java`, `src/main/java/frc/robot/targeting/TargetSolution.java`
- Test: `src/test/java/frc/robot/util/SettleDetectorTest.java`, `src/test/java/frc/robot/targeting/VisionFilterTest.java`, `src/test/java/frc/robot/targeting/TagFieldLayoutLoaderTest.java`, `src/test/java/frc/robot/targeting/TargetSolutionTest.java`

**Interfaces:**
- Produces: `Constants.FieldConstants`, `Constants.VisionConstants.CameraConfig(String name, Transform3d robotToCamera, boolean enabled)`, `VisionConstants.kCameras`, `Constants.AimConstants`, `Constants.ShotConstants`; `SettleDetector(int)`, `update(boolean, boolean)`, `reset()`; `VisionFilter.evaluate(Pose3d, int, double, double, Pose2d, boolean, boolean)` → `Decision(accept, reason, stdDevs)`; `TagFieldLayoutLoader.load()` → `LoadResult(layout, usedFallback)`, `officialLayout()`, `officialLayout(boolean andyMark)`; `TargetSolution.solve(Pose2d, Translation2d)` → `(distanceMeters, bearing, toTarget)`.

- [ ] **Step 1: Copy the four test files from 2026VisionBotoffSeason**

```bash
mkdir -p src/test/java/frc/robot/util src/test/java/frc/robot/targeting
P=../2026VisionBotoffSeason/src/test/java/frc/robot
cp $P/util/SettleDetectorTest.java src/test/java/frc/robot/util/
cp $P/targeting/VisionFilterTest.java $P/targeting/TagFieldLayoutLoaderTest.java $P/targeting/TargetSolutionTest.java src/test/java/frc/robot/targeting/
```

Append to `TagFieldLayoutLoaderTest` (inside the class):

```java
  @Test
  void andyMarkLayoutIsSlightlySmaller() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout(true);
    assertEquals(16.518, layout.getFieldLength(), 0.001);
  }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `./gradlew test --offline`
Expected: compile FAIL (`SettleDetector`, `VisionFilter`, … not found).

- [ ] **Step 3: Copy the ported sources and replace the Vision constants**

```bash
P=../2026VisionBotoffSeason/src/main/java/frc/robot
mkdir -p src/main/java/frc/robot/util src/main/java/frc/robot/targeting
cp $P/util/SettleDetector.java src/main/java/frc/robot/util/
cp $P/targeting/VisionFilter.java $P/targeting/TargetSolution.java $P/targeting/TagFieldLayoutLoader.java src/main/java/frc/robot/targeting/
```

In `TagFieldLayoutLoader`, replace `officialLayout()` with:

```java
  /** The official 2026 REBUILT layout: welded by default, AndyMark when FieldConstants says so. */
  public static AprilTagFieldLayout officialLayout() {
    return officialLayout(FieldConstants.kUseAndyMarkField);
  }

  public static AprilTagFieldLayout officialLayout(boolean andyMark) {
    return AprilTagFieldLayout.loadField(
        andyMark ? AprilTagFields.k2026RebuiltAndymark : AprilTagFields.k2026RebuiltWelded);
  }
```

In `Constants.java`, delete the whole `public static class Vision { … }` block and the imports only it used (`Function`, `PhotonPipelineResult`, `MatBuilder`, `Matrix`, `Nat`, `Transform2d`, `N1`, `N3`, `Radian`, `Feet`, `Inches`). Add imports `edu.wpi.first.math.geometry.Translation2d`, `edu.wpi.first.math.util.Units` and these classes inside `Constants`:

```java
  public static final class FieldConstants {
    private FieldConstants() {}
    /** 2026 REBUILT welded field, from the official AprilTag layout JSON. */
    public static final double kFieldLengthMeters = 16.541;
    public static final double kFieldWidthMeters = 8.069;
    /** false = welded field (WPILib default), true = AndyMark field. Check before each event. */
    public static final boolean kUseAndyMarkField = false;
    /**
     * Lab mode. true = load deploy/practice-layout.json and aim at kLabTargetTagId instead of
     * the hub. Must be false at competition.
     */
    public static final boolean kUsePracticeLayout = false;
    public static final String kPracticeLayoutFile = "practice-layout.json";
    public static final int kLabTargetTagId = 1;
  }

  public static final class VisionConstants {
    private VisionConstants() {}

    /**
     * @param name must match the PhotonVision camera name EXACTLY (a mismatch silently returns
     *     no results; VisionSubsystem reports it 5 s after enable)
     * @param robotToCamera robot center (floor level) to lens, meters; pitch NEGATIVE = tilted up
     * @param enabled false = defined but not constructed
     */
    public record CameraConfig(String name, Transform3d robotToCamera, boolean enabled) {}

    // PLACEHOLDER names and positions until lab steps 3 and 4 (see docs/shooter-vision-guide.html).
    // Front camera: front-right corner under the hood, facing forward.
    public static final CameraConfig kFrontCamera = new CameraConfig(
        "FrontCamera",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.0), Units.inchesToMeters(-11.0), Units.inchesToMeters(15.4)),
            new Rotation3d(0.0, Math.toRadians(-30.0), 0.0)),
        true);
    // Back camera: back-right corner, facing backward (yaw 180°).
    public static final CameraConfig kBackCamera = new CameraConfig(
        "BackCamera",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.0), Units.inchesToMeters(-11.0), Units.inchesToMeters(15.4)),
            new Rotation3d(0.0, Math.toRadians(-30.0), Math.PI)),
        true);
    public static final CameraConfig[] kCameras = {kFrontCamera, kBackCamera};

    public static final double kMaxAmbiguity = 0.2;
    public static final double kMaxPoseJumpMeters = 1.0;
    public static final double kCameraTimeoutSeconds = 5.0;
    public static final double kMaxPoseZMeters = 0.25;
    public static final double kMaxSingleTagDistanceMeters = 4.0;
    public static final double kSingleTagBaseStdDevMeters = 0.20;
    public static final double kMultiTagBaseStdDevMeters = 0.05;
    public static final double kMultiTagThetaStdDevRadians = 0.1;
    public static final double kSingleTagThetaStdDevRadians = 10.0;
  }

  public static final class AimConstants {
    private AimConstants() {}
    // Heading controller (rad -> rad/s). NOT tuned on Arc-imedes.
    public static final double kHeadingP = 5.0;
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.0;
    /** Max aiming turn rate. NOT tuned; lower it if camera latency causes overshoot. */
    public static final double kMaxAimRateRadPerSec = Math.toRadians(270.0);
    public static final double kAimToleranceDeg = 2.0;
    public static final double kSettledRateDegPerSec = 5.0;
    /** No shooting while moving faster than this (no shoot-on-the-move compensation). */
    public static final double kMaxShootSpeedMps = 0.3;
    public static final int kSettledLoops = 5;
    public static final double kMaxVisionAgeToFeedSec = 1.0;
    public static final double kMaxVisionAgeToResetSec = 1.0;
    public static final double kStickDeadband = 0.1;
    public static final double kAutoFeedSeconds = 1.5;
    public static final double kAutoTimeoutSeconds = 4.0;
  }

  public static final class ShotConstants {
    private ShotConstants() {}
    // Where fuel leaves the hood, robot frame, meters. PLACEHOLDER (0, 0) until lab step 5.
    // Front shot leaves the back-right end; back shot leaves the front-right end.
    public static final Translation2d kFrontLaunchPoint = new Translation2d();
    public static final Translation2d kBackLaunchPoint = new Translation2d();
    // Usable hood range, degrees magnitude. PLACEHOLDER until lab step 1.
    public static final double kFrontHoodMinDeg = 0.0;
    public static final double kFrontHoodMaxDeg = 90.0;
    public static final double kBackHoodMinDeg = 0.0;
    public static final double kBackHoodMaxDeg = 90.0;
    public static final double kMaxFlywheelFps = 90.0;
    // Shot table rows {distance ft, hood deg magnitude, flywheel ft/s magnitude}.
    // Seeded from the trench presets; paste calibration rows (Y in CALIBRATION) here.
    public static final double[][] kFrontShots = {{10.0, 88.0, 42.0}};
    public static final double[][] kBackShots = {{10.0, 87.0, 42.0}};
  }
```

`PointToHub` still references `Constants.Vision`; delete it now (it is replaced in Task 6) and remove its imports, field and the `pointToHub` bindings from `RobotContainer` (the X/Start bindings that use it):

```bash
git rm -q src/main/java/frc/robot/commands/PointToHub.java
```

In `RobotContainer`: delete the three `PointToHub` imports, the `pointToHub` field, `joystick.x().whileTrue(pointToHub.onlyIf(...))`, the `joystick.start()` binding, and `updateDistanceWithVision()`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `./gradlew build --offline`
Expected: BUILD SUCCESSFUL, all ported tests pass.

---

### Task 2: ShotDirection, ShotSetpoint, FieldTargets

**Files:**
- Create: `src/main/java/frc/robot/targeting/ShotDirection.java`, `ShotSetpoint.java`, `FieldTargets.java`
- Test: `src/test/java/frc/robot/targeting/ShotDirectionTest.java`, `FieldTargetsTest.java`

**Interfaces:**
- Consumes: `ShotConstants.kFrontLaunchPoint`, `kBackLaunchPoint`; `TagFieldLayoutLoader.officialLayout(boolean)`.
- Produces: `enum ShotDirection { FRONT, BACK }` with `Rotation2d robotHeadingFor(Rotation2d firing)`, `double lateralOffset(Translation2d launchPoint)`, `ShotSetpoint setpoint(double hoodMagDeg, double flywheelMagFps)`, `ShotDirection other()`, `Translation2d launchPoint()`; `record ShotSetpoint(double hoodDeg, double flywheelFps)`; `FieldTargets.hubCenter(AprilTagFieldLayout, Alliance)` → `Optional<Translation2d>`.

- [ ] **Step 1: Write the failing tests**

`ShotDirectionTest.java`:

```java
package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ShotDirectionTest {
  @Test
  void frontFacesTheFiringDirection() {
    assertEquals(30.0, ShotDirection.FRONT.robotHeadingFor(Rotation2d.fromDegrees(30)).getDegrees(), 1e-9);
  }

  @Test
  void backFacesAwayFromTheFiringDirection() {
    assertEquals(-150.0, ShotDirection.BACK.robotHeadingFor(Rotation2d.fromDegrees(30)).getDegrees(), 1e-9);
  }

  @Test
  void setpointSignsFollowTheSpec() {
    assertEquals(new ShotSetpoint(-88.0, 42.0), ShotDirection.FRONT.setpoint(88.0, 42.0));
    assertEquals(new ShotSetpoint(87.0, -42.0), ShotDirection.BACK.setpoint(87.0, 42.0));
  }

  @Test
  void lateralOffsetFlipsForTheBackShot() {
    Translation2d rightSide = new Translation2d(0.0, -0.2);
    assertEquals(-0.2, ShotDirection.FRONT.lateralOffset(rightSide), 1e-9);
    assertEquals(0.2, ShotDirection.BACK.lateralOffset(rightSide), 1e-9);
  }

  @Test
  void otherSwaps() {
    assertEquals(ShotDirection.BACK, ShotDirection.FRONT.other());
    assertEquals(ShotDirection.FRONT, ShotDirection.BACK.other());
  }
}
```

`FieldTargetsTest.java`:

```java
package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import org.junit.jupiter.api.Test;

class FieldTargetsTest {
  @Test
  void weldedBlueHubCenter() {
    Translation2d c = FieldTargets.hubCenter(TagFieldLayoutLoader.officialLayout(false), Alliance.Blue).orElseThrow();
    assertEquals(4.6255, c.getX(), 0.002);
    assertEquals(4.0345, c.getY(), 0.002);
  }

  @Test
  void weldedRedHubCenter() {
    Translation2d c = FieldTargets.hubCenter(TagFieldLayoutLoader.officialLayout(false), Alliance.Red).orElseThrow();
    assertEquals(11.9155, c.getX(), 0.002);
    assertEquals(4.0345, c.getY(), 0.002);
  }

  @Test
  void andyMarkBlueHubCenter() {
    Translation2d c = FieldTargets.hubCenter(TagFieldLayoutLoader.officialLayout(true), Alliance.Blue).orElseThrow();
    assertEquals(4.6115, c.getX(), 0.002);
    assertEquals(4.0215, c.getY(), 0.002);
  }

  @Test
  void layoutWithoutHubTagsGivesNoCenter() {
    AprilTagFieldLayout empty = new AprilTagFieldLayout(List.of(), 16.5, 8.0);
    assertTrue(FieldTargets.hubCenter(empty, Alliance.Blue).isEmpty());
  }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `./gradlew test --offline` — Expected: compile FAIL (`ShotDirection`, `FieldTargets` not found).

- [ ] **Step 3: Implement**

`ShotSetpoint.java`:

```java
package frc.robot.targeting;

/** A signed shooter command: hood angle on the signed scale (deg) and flywheel speed (ft/s). */
public record ShotSetpoint(double hoodDeg, double flywheelFps) {}
```

`ShotDirection.java`:

```java
package frc.robot.targeting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ShotConstants;

/**
 * Which end of Arc-imedes faces the hub.
 *
 * FRONT: flywheel CCW (+), fuel leaves the back-right end of the hood and flies forward; the
 * robot's front faces the hub. BACK: flywheel CW (-), fuel leaves the front-right end and flies
 * backward; the robot's back faces the hub. Hood signs come from the trench presets (front -88°,
 * back +87°) and are confirmed in the lab.
 */
public enum ShotDirection {
  FRONT(Rotation2d.kZero, 1.0, -1.0, 1.0),
  BACK(Rotation2d.fromDegrees(180.0), -1.0, 1.0, -1.0);

  private final Rotation2d m_facingOffset;
  private final double m_flywheelSign;
  private final double m_hoodSign;
  private final double m_lateralSign;

  ShotDirection(Rotation2d facingOffset, double flywheelSign, double hoodSign, double lateralSign) {
    m_facingOffset = facingOffset;
    m_flywheelSign = flywheelSign;
    m_hoodSign = hoodSign;
    m_lateralSign = lateralSign;
  }

  /** Robot heading that sends the fuel along the given field firing direction. */
  public Rotation2d robotHeadingFor(Rotation2d firingDirection) {
    return firingDirection.plus(m_facingOffset);
  }

  /**
   * How far the launch point sits to the LEFT of the firing line through the robot center.
   * Firing forward, left is robot +Y; firing backward, left is robot -Y.
   */
  public double lateralOffset(Translation2d launchPoint) {
    return m_lateralSign * launchPoint.getY();
  }

  /** Turns table magnitudes into the signed command for this direction. */
  public ShotSetpoint setpoint(double hoodMagnitudeDeg, double flywheelMagnitudeFps) {
    return new ShotSetpoint(m_hoodSign * hoodMagnitudeDeg, m_flywheelSign * flywheelMagnitudeFps);
  }

  public ShotDirection other() {
    return this == FRONT ? BACK : FRONT;
  }

  /** Where the fuel leaves the hood for this shot, robot frame, meters. */
  public Translation2d launchPoint() {
    return this == FRONT ? ShotConstants.kFrontLaunchPoint : ShotConstants.kBackLaunchPoint;
  }
}
```

`FieldTargets.java`:

```java
package frc.robot.targeting;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Field points the robot aims at, derived from the AprilTag layout rather than hand-typed. */
public final class FieldTargets {
  private FieldTargets() {}

  /** Tags on each alliance's hub in the 2026 REBUILT layout (all at 1.124 m height). */
  public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};
  public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};

  /**
   * Center of the alliance's hub: the middle of the bounding box of its tags. Empty if any hub
   * tag is missing from the layout (for example, a practice layout).
   */
  public static Optional<Translation2d> hubCenter(AprilTagFieldLayout layout, Alliance alliance) {
    int[] ids = alliance == Alliance.Red ? kRedHubTags : kBlueHubTags;
    double minX = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (int id : ids) {
      Optional<Pose3d> pose = layout.getTagPose(id);
      if (pose.isEmpty()) {
        return Optional.empty();
      }
      minX = Math.min(minX, pose.get().getX());
      maxX = Math.max(maxX, pose.get().getX());
      minY = Math.min(minY, pose.get().getY());
      maxY = Math.max(maxY, pose.get().getY());
    }
    return Optional.of(new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0));
  }
}
```

- [ ] **Step 4: Run to verify pass** — `./gradlew build --offline` → BUILD SUCCESSFUL.

---

### Task 3: HubAim

**Files:**
- Create: `src/main/java/frc/robot/targeting/HubAim.java`
- Test: `src/test/java/frc/robot/targeting/HubAimTest.java`

**Interfaces:**
- Consumes: `TargetSolution.solve`, `ShotDirection`.
- Produces: `HubAim.solve(Pose2d robotPose, Translation2d target, ShotDirection dir, Translation2d launchPoint)` → `HubAim.Solution(ShotDirection direction, Rotation2d robotHeading, Rotation2d headingError, double shotDistanceMeters)`; `HubAim.chooseDirection(Pose2d, Translation2d, Translation2d frontLaunch, Translation2d backLaunch)` → `ShotDirection`; `HubAim.currentShotDistanceMeters(Pose2d, Translation2d, Translation2d)` → `double`; `HubAim.operatorToField(double vx, double vy, boolean red)` → `Translation2d`.

- [ ] **Step 1: Write the failing test**

```java
package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class HubAimTest {
  private static final Translation2d kNone = new Translation2d();
  private static final Translation2d kHub = new Translation2d(10.0, 0.0);

  @Test
  void zeroOffsetFrontAimsStraightAtTarget() {
    var s = HubAim.solve(new Pose2d(0, 0, Rotation2d.fromDegrees(20)), kHub, ShotDirection.FRONT, kNone);
    assertEquals(0.0, s.robotHeading().getDegrees(), 1e-9);
    assertEquals(-20.0, s.headingError().getDegrees(), 1e-9);
    assertEquals(10.0, s.shotDistanceMeters(), 1e-9);
  }

  @Test
  void zeroOffsetBackPointsTheBackAtTarget() {
    var s = HubAim.solve(new Pose2d(0, 0, Rotation2d.kZero), kHub, ShotDirection.BACK, kNone);
    assertEquals(180.0, Math.abs(s.robotHeading().getDegrees()), 1e-9);
  }

  @Test
  void rightSideLaunchPointLinesUpWithTheTarget() {
    for (ShotDirection dir : ShotDirection.values()) {
      Translation2d launch = new Translation2d(0.1, -0.3);
      Pose2d robot = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(50));
      var s = HubAim.solve(robot, kHub, dir, launch);
      Translation2d launchField = robot.getTranslation().plus(launch.rotateBy(s.robotHeading()));
      Rotation2d firing = kHub.minus(launchField).getAngle();
      Rotation2d expectedHeading = dir.robotHeadingFor(firing);
      assertEquals(0.0, expectedHeading.minus(s.robotHeading()).getDegrees(), 1e-6, dir.name());
      assertEquals(kHub.getDistance(launchField), s.shotDistanceMeters(), 1e-9, dir.name());
    }
  }

  @Test
  void headingErrorWrapsAcross180() {
    var s = HubAim.solve(new Pose2d(20, 0, Rotation2d.fromDegrees(170)), kHub, ShotDirection.FRONT, kNone);
    assertEquals(10.0, s.headingError().getDegrees(), 1e-9);
  }

  @Test
  void facingTheHubPicksFront() {
    assertEquals(ShotDirection.FRONT,
        HubAim.chooseDirection(new Pose2d(0, 0, Rotation2d.fromDegrees(30)), kHub, kNone, kNone));
  }

  @Test
  void facingAwayPicksBack() {
    assertEquals(ShotDirection.BACK,
        HubAim.chooseDirection(new Pose2d(0, 0, Rotation2d.fromDegrees(150)), kHub, kNone, kNone));
  }

  @Test
  void exactlySidewaysPicksFront() {
    assertEquals(ShotDirection.FRONT,
        HubAim.chooseDirection(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), kHub, kNone, kNone));
  }

  @Test
  void launchOffsetLargerThanDistanceDoesNotProduceNaN() {
    var s = HubAim.solve(new Pose2d(9.9, 0, Rotation2d.kZero), kHub, ShotDirection.FRONT, new Translation2d(0, -0.5));
    assertFalse(Double.isNaN(s.robotHeading().getRadians()));
    var atTarget = HubAim.solve(new Pose2d(10, 0, Rotation2d.kZero), kHub, ShotDirection.FRONT, kNone);
    assertFalse(Double.isNaN(atTarget.robotHeading().getRadians()));
  }

  @Test
  void currentShotDistanceUsesTheCurrentHeading() {
    Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    // launch point 1 m forward -> at heading 90 it sits at (0, 1)
    assertEquals(Math.hypot(10, 1), HubAim.currentShotDistanceMeters(robot, kHub, new Translation2d(1, 0)), 1e-9);
  }

  @Test
  void operatorToFieldFlipsOnRed() {
    assertEquals(new Translation2d(1, 2), HubAim.operatorToField(1, 2, false));
    assertEquals(new Translation2d(-1, -2), HubAim.operatorToField(1, 2, true));
  }
}
```

- [ ] **Step 2: Run to verify failure** — `./gradlew test --offline` → compile FAIL (`HubAim` not found).

- [ ] **Step 3: Implement**

```java
package frc.robot.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Aim geometry for Arc-imedes' bi-directional shooter. Pure: no hardware, unit-tested.
 *
 * The fuel launches from a point on the right side of the robot, not the center, so the robot
 * turns until the line from that launch point along the firing direction passes through the
 * target: firing = bearing - asin(e / D), where e is the launch point's offset to the left of the
 * firing line and D the center-to-target distance.
 */
public final class HubAim {
  private HubAim() {}

  public record Solution(
      ShotDirection direction,
      Rotation2d robotHeading,
      Rotation2d headingError,
      double shotDistanceMeters) {}

  public static Solution solve(
      Pose2d robotPose, Translation2d target, ShotDirection direction, Translation2d launchPoint) {
    TargetSolution toTarget = TargetSolution.solve(robotPose, target);
    double d = toTarget.distanceMeters();
    double ratio = d < 1e-6 ? 0.0 : MathUtil.clamp(direction.lateralOffset(launchPoint) / d, -1.0, 1.0);
    Rotation2d firing = toTarget.bearing().minus(Rotation2d.fromRadians(Math.asin(ratio)));
    Rotation2d heading = direction.robotHeadingFor(firing);
    Rotation2d error = heading.minus(robotPose.getRotation());
    Translation2d launchField = robotPose.getTranslation().plus(launchPoint.rotateBy(heading));
    return new Solution(direction, heading, error, target.getDistance(launchField));
  }

  /** The end needing the smaller turn. Ties go to FRONT. */
  public static ShotDirection chooseDirection(
      Pose2d robotPose, Translation2d target, Translation2d frontLaunch, Translation2d backLaunch) {
    double front = Math.abs(solve(robotPose, target, ShotDirection.FRONT, frontLaunch).headingError().getRadians());
    double back = Math.abs(solve(robotPose, target, ShotDirection.BACK, backLaunch).headingError().getRadians());
    return back < front - 1e-9 ? ShotDirection.BACK : ShotDirection.FRONT;
  }

  /** Launch point to target at the robot's CURRENT heading (used by calibration). */
  public static double currentShotDistanceMeters(
      Pose2d robotPose, Translation2d target, Translation2d launchPoint) {
    Translation2d launchField =
        robotPose.getTranslation().plus(launchPoint.rotateBy(robotPose.getRotation()));
    return target.getDistance(launchField);
  }

  /**
   * Driver stick velocities are operator-relative; the aim request is blue-origin field-relative.
   * On red the driver faces the other way, so both axes flip.
   */
  public static Translation2d operatorToField(double vx, double vy, boolean redAlliance) {
    return redAlliance ? new Translation2d(-vx, -vy) : new Translation2d(vx, vy);
  }
}
```

- [ ] **Step 4: Run to verify pass** — `./gradlew build --offline` → BUILD SUCCESSFUL.

---

### Task 4: ShotTable

**Files:**
- Create: `src/main/java/frc/robot/targeting/ShotTable.java`
- Test: `src/test/java/frc/robot/targeting/ShotTableTest.java`

**Interfaces:**
- Consumes: `ShotConstants` rows and hood ranges.
- Produces: `new ShotTable(double hoodMinDeg, double hoodMaxDeg)`, `put(double distanceFt, double hoodDeg, double flywheelFps)` → `ShotTable`, `lookup(double distanceFt)` → `ShotTable.Shot(double hoodDeg, double flywheelFps)` (magnitudes), `inRange(double)`, `hoodMinDeg()`, `hoodMaxDeg()`, `static fromRows(double[][], double, double)`, `static forDirection(ShotDirection)`.

- [ ] **Step 1: Write the failing test**

```java
package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotTableTest {
  private static ShotTable table() {
    return new ShotTable(0.0, 90.0).put(6.0, 40.0, 30.0).put(12.0, 60.0, 50.0);
  }

  @Test
  void interpolatesBetweenRows() {
    var shot = table().lookup(9.0);
    assertEquals(50.0, shot.hoodDeg(), 1e-9);
    assertEquals(40.0, shot.flywheelFps(), 1e-9);
  }

  @Test
  void clampsBeyondEnds() {
    assertEquals(40.0, table().lookup(1.0).hoodDeg(), 1e-9);
    assertEquals(50.0, table().lookup(30.0).flywheelFps(), 1e-9);
    assertFalse(table().inRange(30.0));
    assertTrue(table().inRange(9.0));
  }

  @Test
  void singleRowGivesTheSameShotEverywhere() {
    var t = new ShotTable(0.0, 90.0).put(10.0, 88.0, 42.0);
    assertEquals(88.0, t.lookup(3.0).hoodDeg(), 1e-9);
    assertEquals(42.0, t.lookup(20.0).flywheelFps(), 1e-9);
  }

  @Test
  void hoodIsClampedToTheUsableRange() {
    var t = new ShotTable(20.0, 70.0).put(5.0, 10.0, 30.0).put(15.0, 85.0, 60.0);
    assertEquals(20.0, t.lookup(5.0).hoodDeg(), 1e-9);
    assertEquals(70.0, t.lookup(15.0).hoodDeg(), 1e-9);
  }

  @Test
  void emptyTableThrows() {
    assertThrows(IllegalStateException.class, () -> new ShotTable(0.0, 90.0).lookup(5.0));
  }

  @Test
  void fromRowsReadsEveryRow() {
    var t = ShotTable.fromRows(new double[][] {{6.0, 40.0, 30.0}, {12.0, 60.0, 50.0}}, 0.0, 90.0);
    assertEquals(50.0, t.lookup(9.0).hoodDeg(), 1e-9);
  }
}
```

- [ ] **Step 2: Run to verify failure** — compile FAIL (`ShotTable` not found).

- [ ] **Step 3: Implement**

```java
package frc.robot.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ShotConstants;

/**
 * Distance (ft) to hood angle and flywheel speed, as positive magnitudes. Linear interpolation
 * between rows, clamped to the nearest row outside them. ShotDirection applies the signs.
 */
public final class ShotTable {
  public record Shot(double hoodDeg, double flywheelFps) {}

  private final InterpolatingDoubleTreeMap m_hood = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap m_flywheel = new InterpolatingDoubleTreeMap();
  private final double m_hoodMinDeg;
  private final double m_hoodMaxDeg;
  private double m_minDistanceFt = Double.POSITIVE_INFINITY;
  private double m_maxDistanceFt = Double.NEGATIVE_INFINITY;
  private int m_rows;

  public ShotTable(double hoodMinDeg, double hoodMaxDeg) {
    m_hoodMinDeg = hoodMinDeg;
    m_hoodMaxDeg = hoodMaxDeg;
  }

  public ShotTable put(double distanceFt, double hoodDeg, double flywheelFps) {
    m_hood.put(distanceFt, hoodDeg);
    m_flywheel.put(distanceFt, flywheelFps);
    m_minDistanceFt = Math.min(m_minDistanceFt, distanceFt);
    m_maxDistanceFt = Math.max(m_maxDistanceFt, distanceFt);
    m_rows++;
    return this;
  }

  public Shot lookup(double distanceFt) {
    if (m_rows == 0) {
      throw new IllegalStateException("Shot table has no rows");
    }
    double hood = MathUtil.clamp(m_hood.get(distanceFt), m_hoodMinDeg, m_hoodMaxDeg);
    return new Shot(hood, m_flywheel.get(distanceFt));
  }

  /** False when the distance is outside the measured rows (lookup clamps). */
  public boolean inRange(double distanceFt) {
    return distanceFt >= m_minDistanceFt && distanceFt <= m_maxDistanceFt;
  }

  public double hoodMinDeg() {
    return m_hoodMinDeg;
  }

  public double hoodMaxDeg() {
    return m_hoodMaxDeg;
  }

  public static ShotTable fromRows(double[][] rows, double hoodMinDeg, double hoodMaxDeg) {
    ShotTable table = new ShotTable(hoodMinDeg, hoodMaxDeg);
    for (double[] row : rows) {
      table.put(row[0], row[1], row[2]);
    }
    return table;
  }

  public static ShotTable forDirection(ShotDirection direction) {
    return direction == ShotDirection.FRONT
        ? fromRows(ShotConstants.kFrontShots, ShotConstants.kFrontHoodMinDeg, ShotConstants.kFrontHoodMaxDeg)
        : fromRows(ShotConstants.kBackShots, ShotConstants.kBackHoodMinDeg, ShotConstants.kBackHoodMaxDeg);
  }
}
```

- [ ] **Step 4: Run to verify pass** — `./gradlew build --offline` → BUILD SUCCESSFUL.

---

### Task 5: ShotGate

**Files:**
- Create: `src/main/java/frc/robot/targeting/ShotGate.java`
- Test: `src/test/java/frc/robot/targeting/ShotGateTest.java`

**Interfaces:**
- Produces: `ShotGate.evaluate(boolean aimed, boolean stopped, boolean settled, boolean shooterAtTarget, double secondsSinceVision, double maxVisionAgeSec)` → `ShotGate.Status { READY, NOT_AIMED, MOVING, SETTLING, SHOOTER_NOT_READY, VISION_STALE }`.

- [ ] **Step 1: Write the failing test**

```java
package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.targeting.ShotGate.Status;
import org.junit.jupiter.api.Test;

class ShotGateTest {
  private static Status eval(boolean aimed, boolean stopped, boolean settled, boolean atTarget, double visionAge) {
    return ShotGate.evaluate(aimed, stopped, settled, atTarget, visionAge, 1.0);
  }

  @Test
  void allConditionsMetIsReady() {
    assertEquals(Status.READY, eval(true, true, true, true, 0.2));
  }

  @Test
  void eachConditionBlocksOnItsOwn() {
    assertEquals(Status.NOT_AIMED, eval(false, true, true, true, 0.2));
    assertEquals(Status.MOVING, eval(true, false, true, true, 0.2));
    assertEquals(Status.SETTLING, eval(true, true, false, true, 0.2));
    assertEquals(Status.SHOOTER_NOT_READY, eval(true, true, true, false, 0.2));
    assertEquals(Status.VISION_STALE, eval(true, true, true, true, 1.5));
  }

  @Test
  void neverSeenVisionIsStale() {
    assertEquals(Status.VISION_STALE, eval(true, true, true, true, Double.POSITIVE_INFINITY));
  }
}
```

- [ ] **Step 2: Run to verify failure** — compile FAIL.

- [ ] **Step 3: Implement**

```java
package frc.robot.targeting;

/** Whether align-and-shoot may feed fuel, and if not, the first reason why not. Pure. */
public final class ShotGate {
  private ShotGate() {}

  public enum Status { READY, NOT_AIMED, MOVING, SETTLING, SHOOTER_NOT_READY, VISION_STALE }

  public static Status evaluate(
      boolean aimed,
      boolean stopped,
      boolean settled,
      boolean shooterAtTarget,
      double secondsSinceVision,
      double maxVisionAgeSec) {
    if (!aimed) {
      return Status.NOT_AIMED;
    }
    if (!stopped) {
      return Status.MOVING;
    }
    if (!settled) {
      return Status.SETTLING;
    }
    if (!shooterAtTarget) {
      return Status.SHOOTER_NOT_READY;
    }
    if (!(secondsSinceVision <= maxVisionAgeSec)) {
      return Status.VISION_STALE;
    }
    return Status.READY;
  }
}
```

- [ ] **Step 4: Run to verify pass** — `./gradlew build --offline` → BUILD SUCCESSFUL.

---

### Task 6: VisionSwerveDrivetrain + VisionSubsystem wired into RobotContainer

**Files:**
- Create: `src/main/java/frc/robot/subsystems/VisionSwerveDrivetrain.java`, `src/main/java/frc/robot/subsystems/VisionSubsystem.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java` (drivetrain field, vision field, Start binding)

**Interfaces:**
- Consumes: Task 1 constants, `VisionFilter`, `TagFieldLayoutLoader`, `FieldTargets`.
- Produces: `VisionSwerveDrivetrain` (extends `CommandSwerveDrivetrain`): `getRobotPose()`, `getYawRateDegPerSec()`, `getTranslationalSpeedMps()`, `driveFacingAngle(double vx, double vy, Rotation2d heading)`, `driveFieldRelative(double vx, double vy, double omega)`, `stopDrive()`. `VisionSubsystem(VisionSwerveDrivetrain)`: `getSecondsSinceAccepted()`, `isVisionFresh(double maxAgeSec)`, `getAimTarget(Optional<Alliance>)` → `Optional<Translation2d>`, `resetTranslationFromVision()` → `boolean`, `hasTarget()`, `isHealthy()`.

- [ ] **Step 1: Port VisionSwerveDrivetrain**

Copy `../2026VisionBotoffSeason/src/main/java/frc/robot/subsystems/VisionSwerveDrivetrain.java`, then remove `m_routineRequest`, `driveRoutineFacingAngle(...)`, the `RoutineConstants` import and its two constructor lines. Keep `m_aimRequest` (BlueAlliance perspective, zero deadbands), `m_fieldRelativeRequest`, `m_idleRequest`, and the methods listed above.

- [ ] **Step 2: Port VisionSubsystem with additions**

Copy `../2026VisionBotoffSeason/src/main/java/frc/robot/subsystems/VisionSubsystem.java` and make these changes:

1. Add a field `private Pose2d m_lastAcceptedPose;` and, in `processResult(...)` right after `m_drivetrain.addVisionMeasurement(...)`:

```java
    if (solve.estimate().timestampSeconds >= m_lastAcceptedTimestamp) {
      m_lastAcceptedPose = pose.toPose2d();
    }
```

(place it before the `m_lastAcceptedTimestamp = Math.max(...)` line).

2. Add these methods (imports: `edu.wpi.first.wpilibj.DriverStation.Alliance`, `frc.robot.Constants.AimConstants`, `frc.robot.targeting.FieldTargets`):

```java
  /**
   * What to aim at: the alliance hub's center on the official field, or the lab tag when the
   * practice layout is on. Empty when the alliance is unknown, the layout fell back, or the
   * target isn't in the layout - commands then don't rotate.
   */
  public Optional<Translation2d> getAimTarget(Optional<Alliance> alliance) {
    if (m_loadResult.usedFallback()) {
      return Optional.empty();
    }
    if (FieldConstants.kUsePracticeLayout) {
      return getTargetPosition(FieldConstants.kLabTargetTagId);
    }
    return alliance.flatMap(a -> FieldTargets.hubCenter(m_layout, a));
  }

  public boolean isVisionFresh(double maxAgeSec) {
    return getSecondsSinceAccepted() <= maxAgeSec;
  }

  /**
   * Snap the pose translation to the newest accepted vision pose (Start button). Refuses when
   * vision is older than kMaxVisionAgeToResetSec. Heading is left to the gyro.
   */
  public boolean resetTranslationFromVision() {
    if (m_lastAcceptedPose == null || !isVisionFresh(AimConstants.kMaxVisionAgeToResetSec)) {
      System.out.println("Vision reset refused: no accepted vision pose in the last "
          + AimConstants.kMaxVisionAgeToResetSec + "s");
      return false;
    }
    m_drivetrain.resetTranslation(m_lastAcceptedPose.getTranslation());
    return true;
  }
```

- [ ] **Step 3: Wire into RobotContainer**

Replace the drivetrain field and add vision:

```java
  public final VisionSwerveDrivetrain drivetrain = new VisionSwerveDrivetrain();
  private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
```

Replace the `CommandSwerveDrivetrain` import with `frc.robot.subsystems.VisionSwerveDrivetrain` and `frc.robot.subsystems.VisionSubsystem`. Add the Start binding:

```java
    joystick.start().onFalse(Commands.runOnce(vision::resetTranslationFromVision));
```

- [ ] **Step 4: Build** — `./gradlew build --offline` → BUILD SUCCESSFUL.

---

### Task 7: AlignToHubCommand, AlignAndShootCommand, ShootCommand + X/B bindings + auto named command

**Files:**
- Create: `src/main/java/frc/robot/commands/AlignToHubCommand.java`, `AlignAndShootCommand.java`, `ShootCommand.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`

**Interfaces:**
- Consumes: `HubAim`, `ShotTable`, `ShotGate`, `SettleDetector`, `VisionSubsystem`, `VisionSwerveDrivetrain`, `ShooterSubSystem.setHoodAngle(Angle)`, `setFlywheelVelocity(LinearVelocity)`, `idleHood()`, `isAtTarget()`, `Hopper.setRollers(RollerState)`.
- Produces: `new AlignToHubCommand(VisionSwerveDrivetrain, VisionSubsystem, DoubleSupplier vxOperatorMps, DoubleSupplier vyOperatorMps)`; `new AlignAndShootCommand(VisionSwerveDrivetrain, VisionSubsystem, ShooterSubSystem, Hopper, ShotTable front, ShotTable back, DoubleSupplier vx, DoubleSupplier vy)` with `double getSecondsFed()`; `new ShootCommand(ShooterSubSystem, Hopper, Supplier<ShotSetpoint>)`.

- [ ] **Step 1: ShootCommand**

```java
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.targeting.ShotSetpoint;
import java.util.function.Supplier;

/**
 * Hold-to-run shot without aiming: spin to the supplied setpoint, feed whenever the shooter is
 * on target, stop everything on release. Used by MANUAL (shot table at the driver's distance)
 * and CALIBRATION (hand-set values).
 */
public class ShootCommand extends Command {
  private final ShooterSubSystem m_shooter;
  private final Hopper m_hopper;
  private final Supplier<ShotSetpoint> m_setpoint;

  public ShootCommand(ShooterSubSystem shooter, Hopper hopper, Supplier<ShotSetpoint> setpoint) {
    m_shooter = shooter;
    m_hopper = hopper;
    m_setpoint = setpoint;
    addRequirements(shooter, hopper);
  }

  @Override
  public void execute() {
    ShotSetpoint sp = m_setpoint.get();
    m_shooter.setHoodAngle(Degrees.of(sp.hoodDeg()));
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(sp.flywheelFps()));
    boolean ready = m_shooter.isAtTarget();
    m_hopper.setRollers(ready ? RollerState.FORWARD : RollerState.OFF);
    DogLog.log("Shooter/ReadyToFire", ready);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
    m_shooter.idleHood();
    m_hopper.setRollers(RollerState.OFF);
    DogLog.log("Shooter/ReadyToFire", false);
  }
}
```

- [ ] **Step 2: AlignToHubCommand**

```java
package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.targeting.HubAim;
import frc.robot.targeting.ShotDirection;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Hold X: turn whichever end of the robot needs less rotation toward the hub while the driver
 * keeps translating with the left stick. Aims from the fused pose, never raw camera bearings.
 * The direction is chosen once at press time so it can't flip mid-turn.
 */
public class AlignToHubCommand extends Command {
  protected final VisionSwerveDrivetrain m_drivetrain;
  protected final VisionSubsystem m_vision;
  private final DoubleSupplier m_vxOperator;
  private final DoubleSupplier m_vyOperator;

  protected Optional<Translation2d> m_target = Optional.empty();
  protected ShotDirection m_direction = ShotDirection.FRONT;
  protected HubAim.Solution m_solution;
  private boolean m_red;

  public AlignToHubCommand(
      VisionSwerveDrivetrain drivetrain,
      VisionSubsystem vision,
      DoubleSupplier vxOperatorMps,
      DoubleSupplier vyOperatorMps) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_vxOperator = vxOperatorMps;
    m_vyOperator = vyOperatorMps;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    m_red = alliance.orElse(Alliance.Blue) == Alliance.Red;
    m_target = m_vision.getAimTarget(alliance);
    m_solution = null;
    if (m_target.isPresent()) {
      m_direction = HubAim.chooseDirection(
          m_drivetrain.getRobotPose(), m_target.get(),
          ShotDirection.FRONT.launchPoint(), ShotDirection.BACK.launchPoint());
      DogLog.log("Aim/Direction", m_direction.name());
    } else {
      DogLog.log("Aim/Direction", alliance.isEmpty() ? "NO_ALLIANCE" : "NO_TARGET");
    }
  }

  @Override
  public void execute() {
    Translation2d field = HubAim.operatorToField(
        m_vxOperator.getAsDouble(), m_vyOperator.getAsDouble(), m_red);
    if (m_target.isEmpty()) {
      m_solution = null;
      m_drivetrain.driveFieldRelative(field.getX(), field.getY(), 0.0);
      return;
    }
    Pose2d pose = m_drivetrain.getRobotPose();
    m_solution = HubAim.solve(pose, m_target.get(), m_direction, m_direction.launchPoint());
    m_drivetrain.driveFacingAngle(field.getX(), field.getY(), m_solution.robotHeading());

    double errorDeg = m_solution.headingError().getDegrees();
    DogLog.log("Aim/HeadingErrorDeg", errorDeg);
    DogLog.log("Aim/DistanceFt", edu.wpi.first.math.util.Units.metersToFeet(m_solution.shotDistanceMeters()));
    DogLog.log("Aim/Aimed", Math.abs(errorDeg) <= AimConstants.kAimToleranceDeg);
    DogLog.log("Aim/VisionFresh", m_vision.isVisionFresh(AimConstants.kMaxVisionAgeToFeedSec));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
  }
}
```

- [ ] **Step 3: AlignAndShootCommand**

```java
package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.targeting.ShotDirection;
import frc.robot.targeting.ShotGate;
import frc.robot.targeting.ShotSetpoint;
import frc.robot.targeting.ShotTable;
import frc.robot.util.SettleDetector;
import java.util.function.DoubleSupplier;

/**
 * Hold B: align (as AlignToHubCommand), set hood and flywheel from the shot table for the live
 * distance, and feed only when aimed, settled, stopped, on target and vision is fresh.
 */
public class AlignAndShootCommand extends AlignToHubCommand {
  private final ShooterSubSystem m_shooter;
  private final Hopper m_hopper;
  private final ShotTable m_frontTable;
  private final ShotTable m_backTable;
  private final SettleDetector m_settle = new SettleDetector(AimConstants.kSettledLoops);
  private double m_secondsFed;
  private double m_lastTimestamp;

  public AlignAndShootCommand(
      VisionSwerveDrivetrain drivetrain,
      VisionSubsystem vision,
      ShooterSubSystem shooter,
      Hopper hopper,
      ShotTable frontTable,
      ShotTable backTable,
      DoubleSupplier vxOperatorMps,
      DoubleSupplier vyOperatorMps) {
    super(drivetrain, vision, vxOperatorMps, vyOperatorMps);
    m_shooter = shooter;
    m_hopper = hopper;
    m_frontTable = frontTable;
    m_backTable = backTable;
    addRequirements(shooter, hopper);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_settle.reset();
    m_secondsFed = 0.0;
    m_lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    super.execute();
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastTimestamp;
    m_lastTimestamp = now;

    if (m_solution == null) {
      m_hopper.setRollers(RollerState.OFF);
      DogLog.log("Shooter/ReadyToFire", false);
      DogLog.log("Shooter/BlockedBy", "NO_TARGET");
      return;
    }

    ShotTable table = m_direction == ShotDirection.FRONT ? m_frontTable : m_backTable;
    double distanceFt = Units.metersToFeet(m_solution.shotDistanceMeters());
    ShotTable.Shot shot = table.lookup(distanceFt);
    ShotSetpoint sp = m_direction.setpoint(shot.hoodDeg(), shot.flywheelFps());
    m_shooter.setHoodAngle(Degrees.of(sp.hoodDeg()));
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(sp.flywheelFps()));

    boolean aimed = Math.abs(m_solution.headingError().getDegrees()) <= AimConstants.kAimToleranceDeg;
    boolean stopped = Math.abs(m_drivetrain.getYawRateDegPerSec()) < AimConstants.kSettledRateDegPerSec
        && m_drivetrain.getTranslationalSpeedMps() < AimConstants.kMaxShootSpeedMps;
    boolean settled = m_settle.update(aimed, stopped);
    ShotGate.Status status = ShotGate.evaluate(
        aimed, stopped, settled, m_shooter.isAtTarget(),
        m_vision.getSecondsSinceAccepted(), AimConstants.kMaxVisionAgeToFeedSec);

    boolean feed = status == ShotGate.Status.READY;
    m_hopper.setRollers(feed ? RollerState.FORWARD : RollerState.OFF);
    if (feed) {
      m_secondsFed += dt;
    }

    DogLog.log("Shooter/ReadyToFire", feed);
    DogLog.log("Shooter/BlockedBy", status.name());
    DogLog.log("Shooter/OutOfRange", !table.inRange(distanceFt));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
    m_shooter.idleHood();
    m_hopper.setRollers(RollerState.OFF);
    DogLog.log("Shooter/ReadyToFire", false);
  }

  /** Seconds the hopper has fed during this run (used by the auto named command). */
  public double getSecondsFed() {
    return m_secondsFed;
  }
}
```

- [ ] **Step 4: Bindings and named command in RobotContainer**

Add fields:

```java
  private final ShotTable frontShots = ShotTable.forDirection(ShotDirection.FRONT);
  private final ShotTable backShots = ShotTable.forDirection(ShotDirection.BACK);
```

Add helpers (operator-relative stick velocities, same sign and inversion as the default drive):

```java
  private double stickVx() {
    return -MathUtil.applyDeadband(joystick.getLeftY(), AimConstants.kStickDeadband) * MaxSpeed * controlsInverted;
  }

  private double stickVy() {
    return -MathUtil.applyDeadband(joystick.getLeftX(), AimConstants.kStickDeadband) * MaxSpeed * controlsInverted;
  }
```

In `configureBindings()` remove the MANUAL "point wheels" X binding and add:

```java
    joystick.x().whileTrue(new AlignToHubCommand(drivetrain, vision, this::stickVx, this::stickVy));
    joystick.b().whileTrue(new AlignAndShootCommand(
        drivetrain, vision, shooter, hopper, frontShots, backShots, this::stickVx, this::stickVy));
```

Register the auto named command in the constructor (before `configureBindings()`):

```java
    AlignAndShootCommand autoShot = new AlignAndShootCommand(
        drivetrain, vision, shooter, hopper, frontShots, backShots, () -> 0.0, () -> 0.0);
    NamedCommands.registerCommand("alignAndShoot",
        autoShot.until(() -> autoShot.getSecondsFed() >= AimConstants.kAutoFeedSeconds)
            .withTimeout(AimConstants.kAutoTimeoutSeconds));
```

Delete the now-unused `point` request field.

- [ ] **Step 5: Build** — `./gradlew build --offline` → BUILD SUCCESSFUL.

---

### Task 8: Calibration mode (state, dashboard, recorder) + MANUAL/CAL shooter bindings, AUTO mode removed

**Files:**
- Create: `src/main/java/frc/robot/calibration/CalibrationState.java`, `CalibrationDashboard.java`, `ShotRecorder.java`
- Test: `src/test/java/frc/robot/calibration/CalibrationStateTest.java`, `ShotRecorderTest.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`, `src/main/java/frc/robot/Robot.java`
- Delete: `src/main/java/frc/robot/utils/ShooterCalculator.java`

**Interfaces:**
- Consumes: `ShotTable`, `ShotDirection`, `ShotSetpoint`, `HubAim.currentShotDistanceMeters`, `VisionSubsystem.getAimTarget`, `isVisionFresh`, `ShootCommand`.
- Produces: `CalibrationState(ShotTable front, ShotTable back)` with `direction()`, `hoodDeg()`, `flywheelFps()`, `useManualDistance()`, `manualDistanceFt()`, `resetToTable(double ft)`, `toggleDirection(double ft)`, `stepHood(double)`, `stepFlywheel(double)`, `setHoodDeg(double)`, `setFlywheelFps(double)`, `setUseManualDistance(boolean)`, `setManualDistanceFt(double)`, `setpoint()`; `CalibrationDashboard(CalibrationState)` with `update(double distanceFt, String source, boolean visionFresh, double actualHoodDeg, double actualFlywheelFps)`, `setLastSaved(String)`; `ShotRecorder(Path csv)` with `record(Row)` → `String`, static `refusalReason(double distanceFt, boolean useManual, boolean visionFresh)` → `Optional<String>`, `csvLine(Row)`, `pasteLine(Row)`, `kCsvHeader`; `RobotContainer.periodic()`.

- [ ] **Step 1: Write the failing tests**

`CalibrationStateTest.java`:

```java
package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.targeting.ShotDirection;
import frc.robot.targeting.ShotSetpoint;
import frc.robot.targeting.ShotTable;
import org.junit.jupiter.api.Test;

class CalibrationStateTest {
  private static CalibrationState state() {
    ShotTable front = new ShotTable(10.0, 80.0).put(6.0, 40.0, 30.0).put(12.0, 60.0, 50.0);
    ShotTable back = new ShotTable(20.0, 85.0).put(6.0, 45.0, 35.0).put(12.0, 65.0, 55.0);
    return new CalibrationState(front, back);
  }

  @Test
  void startsFrontAtTheTableEntry() {
    var s = state();
    s.resetToTable(9.0);
    assertEquals(ShotDirection.FRONT, s.direction());
    assertEquals(50.0, s.hoodDeg(), 1e-9);
    assertEquals(40.0, s.flywheelFps(), 1e-9);
  }

  @Test
  void toggleSwitchesDirectionAndResetsToThatTable() {
    var s = state();
    s.toggleDirection(9.0);
    assertEquals(ShotDirection.BACK, s.direction());
    assertEquals(55.0, s.hoodDeg(), 1e-9);
    assertEquals(new ShotSetpoint(55.0, -45.0), s.setpoint());
  }

  @Test
  void stepsChangeMagnitudes() {
    var s = state();
    s.resetToTable(6.0);
    s.stepHood(2.5);
    s.stepFlywheel(-2.0);
    assertEquals(new ShotSetpoint(-42.5, 28.0), s.setpoint());
  }

  @Test
  void typedValuesAreClamped() {
    var s = state();
    s.setHoodDeg(120.0);
    assertEquals(80.0, s.hoodDeg(), 1e-9);
    s.setHoodDeg(-5.0);
    assertEquals(10.0, s.hoodDeg(), 1e-9);
    s.setFlywheelFps(-10.0);
    assertEquals(0.0, s.flywheelFps(), 1e-9);
    s.setFlywheelFps(500.0);
    assertEquals(90.0, s.flywheelFps(), 1e-9);
    s.setManualDistanceFt(-3.0);
    assertEquals(0.0, s.manualDistanceFt(), 1e-9);
  }
}
```

`ShotRecorderTest.java`:

```java
package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.targeting.ShotDirection;
import org.junit.jupiter.api.Test;

class ShotRecorderTest {
  private static final ShotRecorder.Row kRow = new ShotRecorder.Row(
      12.5, ShotDirection.BACK, 9.0, "VISION", 64.5, 64.0, -38.0, -37.6, true);

  @Test
  void csvLineMatchesHeaderColumns() {
    assertEquals(ShotRecorder.kCsvHeader.split(",").length, ShotRecorder.csvLine(kRow).split(",").length);
    assertEquals("12.50,BACK,9.00,VISION,64.5,64.0,-38.0,-37.6,true", ShotRecorder.csvLine(kRow));
  }

  @Test
  void pasteLineUsesMagnitudesInTableOrder() {
    assertEquals("{9.0, 64.5, 38.0}, // BACK: distance ft, hood deg, flywheel ft/s", ShotRecorder.pasteLine(kRow));
  }

  @Test
  void refusesWithoutTrustedDistance() {
    assertTrue(ShotRecorder.refusalReason(Double.NaN, false, true).isPresent());
    assertTrue(ShotRecorder.refusalReason(9.0, false, false).isPresent());
    assertTrue(ShotRecorder.refusalReason(9.0, true, false).isEmpty());
    assertTrue(ShotRecorder.refusalReason(9.0, false, true).isEmpty());
  }
}
```

- [ ] **Step 2: Run to verify failure** — compile FAIL.

- [ ] **Step 3: Implement CalibrationState**

```java
package frc.robot.calibration;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShotConstants;
import frc.robot.targeting.ShotDirection;
import frc.robot.targeting.ShotSetpoint;
import frc.robot.targeting.ShotTable;

/** CALIBRATION-mode values: direction, hood and flywheel magnitudes, distance source. */
public class CalibrationState {
  private final ShotTable m_front;
  private final ShotTable m_back;
  private ShotDirection m_direction = ShotDirection.FRONT;
  private double m_hoodDeg;
  private double m_flywheelFps;
  private boolean m_useManualDistance;
  private double m_manualDistanceFt = 10.0;

  public CalibrationState(ShotTable front, ShotTable back) {
    m_front = front;
    m_back = back;
    resetToTable(m_manualDistanceFt);
  }

  /** Start from the table's entry for this distance and the current direction. */
  public void resetToTable(double distanceFt) {
    ShotTable.Shot shot = table().lookup(Double.isNaN(distanceFt) ? m_manualDistanceFt : distanceFt);
    setHoodDeg(shot.hoodDeg());
    setFlywheelFps(shot.flywheelFps());
  }

  public void toggleDirection(double distanceFt) {
    m_direction = m_direction.other();
    resetToTable(distanceFt);
  }

  public void stepHood(double deltaDeg) {
    setHoodDeg(m_hoodDeg + deltaDeg);
  }

  public void stepFlywheel(double deltaFps) {
    setFlywheelFps(m_flywheelFps + deltaFps);
  }

  public void setHoodDeg(double hoodDeg) {
    m_hoodDeg = MathUtil.clamp(hoodDeg, table().hoodMinDeg(), table().hoodMaxDeg());
  }

  public void setFlywheelFps(double flywheelFps) {
    m_flywheelFps = MathUtil.clamp(flywheelFps, 0.0, ShotConstants.kMaxFlywheelFps);
  }

  public void setUseManualDistance(boolean useManual) {
    m_useManualDistance = useManual;
  }

  public void setManualDistanceFt(double distanceFt) {
    m_manualDistanceFt = Math.max(0.0, distanceFt);
  }

  public ShotSetpoint setpoint() {
    return m_direction.setpoint(m_hoodDeg, m_flywheelFps);
  }

  public ShotDirection direction() {
    return m_direction;
  }

  public double hoodDeg() {
    return m_hoodDeg;
  }

  public double flywheelFps() {
    return m_flywheelFps;
  }

  public boolean useManualDistance() {
    return m_useManualDistance;
  }

  public double manualDistanceFt() {
    return m_manualDistanceFt;
  }

  private ShotTable table() {
    return m_direction == ShotDirection.FRONT ? m_front : m_back;
  }
}
```

- [ ] **Step 4: Implement ShotRecorder**

```java
package frc.robot.calibration;

import dev.doglog.DogLog;
import frc.robot.targeting.ShotDirection;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.Locale;
import java.util.Optional;

/** Saves calibration shots that scored, as shot-table rows. */
public class ShotRecorder {
  public record Row(
      double timeSec,
      ShotDirection direction,
      double distanceFt,
      String distanceSource,
      double hoodSetDeg,
      double hoodActualDeg,
      double flywheelSetFps,
      double flywheelActualFps,
      boolean visionFresh) {}

  public static final String kCsvHeader =
      "timeSec,direction,distanceFt,distanceSource,hoodSetDeg,hoodActualDeg,"
          + "flywheelSetFps,flywheelActualFps,visionFresh";

  private final Path m_csvPath;
  private boolean m_warned;

  public ShotRecorder(Path csvPath) {
    m_csvPath = csvPath;
  }

  /** Why a row can't be saved, or empty if it can. */
  public static Optional<String> refusalReason(double distanceFt, boolean useManualDistance, boolean visionFresh) {
    if (Double.isNaN(distanceFt)) {
      return Optional.of("no distance: no aim target and manual distance is off");
    }
    if (!useManualDistance && !visionFresh) {
      return Optional.of("vision not fresh: wait for tags or turn on Cal/UseManualDistance");
    }
    return Optional.empty();
  }

  public static String csvLine(Row r) {
    return String.format(Locale.US, "%.2f,%s,%.2f,%s,%.1f,%.1f,%.1f,%.1f,%b",
        r.timeSec(), r.direction().name(), r.distanceFt(), r.distanceSource(),
        r.hoodSetDeg(), r.hoodActualDeg(), r.flywheelSetFps(), r.flywheelActualFps(), r.visionFresh());
  }

  /** A row to paste into ShotConstants.kFrontShots / kBackShots. */
  public static String pasteLine(Row r) {
    return String.format(Locale.US, "{%.1f, %.1f, %.1f}, // %s: distance ft, hood deg, flywheel ft/s",
        r.distanceFt(), Math.abs(r.hoodSetDeg()), Math.abs(r.flywheelSetFps()), r.direction().name());
  }

  /** Log, append to CSV and print the paste line. Returns the text for Cal/LastSaved. */
  public String record(Row r) {
    DogLog.log("Cal/Saved/Direction", r.direction().name());
    DogLog.log("Cal/Saved/DistanceFt", r.distanceFt());
    DogLog.log("Cal/Saved/HoodDeg", r.hoodSetDeg());
    DogLog.log("Cal/Saved/FlywheelFps", r.flywheelSetFps());
    try {
      Files.createDirectories(m_csvPath.getParent());
      boolean isNew = !Files.exists(m_csvPath);
      String text = (isNew ? kCsvHeader + "\n" : "") + csvLine(r) + "\n";
      Files.writeString(m_csvPath, text, StandardCharsets.UTF_8,
          StandardOpenOption.CREATE, StandardOpenOption.APPEND);
    } catch (IOException e) {
      if (!m_warned) {
        m_warned = true;
        System.out.println("ShotRecorder: could not write " + m_csvPath + ": " + e.getMessage());
      }
    }
    String paste = pasteLine(r);
    System.out.println("SHOT " + paste);
    return paste;
  }
}
```

- [ ] **Step 5: Implement CalibrationDashboard**

```java
package frc.robot.calibration;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/**
 * Publishes the calibration values under "Cal/" and applies edits typed on a dashboard.
 * An edit is any value that differs from what this class last published.
 */
public class CalibrationDashboard {
  private final CalibrationState m_state;
  private final DoubleEntry m_hood;
  private final DoubleEntry m_flywheel;
  private final DoubleEntry m_manualDistance;
  private final BooleanEntry m_useManual;
  private final StringPublisher m_direction;
  private final DoublePublisher m_distance;
  private final StringPublisher m_distanceSource;
  private final BooleanPublisher m_visionFresh;
  private final DoublePublisher m_actualHood;
  private final DoublePublisher m_actualFlywheel;
  private final StringPublisher m_lastSaved;

  private boolean m_first = true;
  private double m_lastHood;
  private double m_lastFlywheel;
  private double m_lastManualDistance;
  private boolean m_lastUseManual;

  public CalibrationDashboard(CalibrationState state) {
    m_state = state;
    NetworkTable t = NetworkTableInstance.getDefault().getTable("Cal");
    m_hood = t.getDoubleTopic("HoodDeg").getEntry(0.0);
    m_flywheel = t.getDoubleTopic("FlywheelFps").getEntry(0.0);
    m_manualDistance = t.getDoubleTopic("ManualDistanceFt").getEntry(0.0);
    m_useManual = t.getBooleanTopic("UseManualDistance").getEntry(false);
    m_direction = t.getStringTopic("Direction").publish();
    m_distance = t.getDoubleTopic("DistanceFt").publish();
    m_distanceSource = t.getStringTopic("DistanceSource").publish();
    m_visionFresh = t.getBooleanTopic("VisionFresh").publish();
    m_actualHood = t.getDoubleTopic("ActualHoodDeg").publish();
    m_actualFlywheel = t.getDoubleTopic("ActualFlywheelFps").publish();
    m_lastSaved = t.getStringTopic("LastSaved").publish();
    m_lastSaved.set("");
  }

  public void update(
      double distanceFt, String source, boolean visionFresh, double actualHoodDeg, double actualFlywheelFps) {
    if (!m_first) {
      double hood = m_hood.get();
      if (hood != m_lastHood) {
        m_state.setHoodDeg(hood);
      }
      double flywheel = m_flywheel.get();
      if (flywheel != m_lastFlywheel) {
        m_state.setFlywheelFps(flywheel);
      }
      double manual = m_manualDistance.get();
      if (manual != m_lastManualDistance) {
        m_state.setManualDistanceFt(manual);
      }
      boolean useManual = m_useManual.get();
      if (useManual != m_lastUseManual) {
        m_state.setUseManualDistance(useManual);
      }
    }
    m_first = false;

    m_lastHood = m_state.hoodDeg();
    m_lastFlywheel = m_state.flywheelFps();
    m_lastManualDistance = m_state.manualDistanceFt();
    m_lastUseManual = m_state.useManualDistance();
    m_hood.set(m_lastHood);
    m_flywheel.set(m_lastFlywheel);
    m_manualDistance.set(m_lastManualDistance);
    m_useManual.set(m_lastUseManual);

    m_direction.set(m_state.direction().name());
    m_distance.set(distanceFt);
    m_distanceSource.set(source);
    m_visionFresh.set(visionFresh);
    m_actualHood.set(actualHoodDeg);
    m_actualFlywheel.set(actualFlywheelFps);
  }

  public void setLastSaved(String text) {
    m_lastSaved.set(text);
  }
}
```

- [ ] **Step 6: RobotContainer: remove AUTO mode and the old shooter loop; bind MANUAL and CALIBRATION**

Remove fields `targetFlywheelVelocity`, `targetHoodAngle`, `side`, `isShooting`; methods `updateShooter()`, `updateShooterInCalibration()`, `updateDistanceWithVision()`; enum `Side`; `Mode.AUTO` (enum becomes `MANUAL, CALIBRATION`); the Y mode-toggle binding; both right-trigger bindings; the four bumper bindings; the CALIBRATION branches inside the D-pad left/right/down bindings (outside-CAL presets stay); imports `ShooterCalculator`, `LEFT_HOOD_BASE_ANGLE`. Delete `src/main/java/frc/robot/utils/ShooterCalculator.java`.

Add fields:

```java
  private ShotDirection manualDirection = ShotDirection.FRONT;
  private final CalibrationState calibration = new CalibrationState(frontShots, backShots);
  private final CalibrationDashboard calibrationDashboard = new CalibrationDashboard(calibration);
  private final ShotRecorder shotRecorder = new ShotRecorder(
      Filesystem.getOperatingDirectory().toPath().resolve("logs").resolve("shot-calibration.csv"));

  private record CalDistance(double feet, String source) {}
```

Add bindings in `configureBindings()` (`Trigger` from `edu.wpi.first.wpilibj2.command.button.Trigger`):

```java
    Trigger manual = new Trigger(() -> mode == Mode.MANUAL);
    Trigger calibrating = new Trigger(() -> mode == Mode.CALIBRATION);

    joystick.povUp().onTrue(Commands.runOnce(() -> {
      if (mode == Mode.CALIBRATION) {
        mode = Mode.MANUAL;
      } else {
        mode = Mode.CALIBRATION;
        calibration.resetToTable(calibrationDistance().feet());
      }
    }));

    joystick.rightTrigger().and(manual).whileTrue(new ShootCommand(shooter, hopper, () -> {
      ShotTable table = manualDirection == ShotDirection.FRONT ? frontShots : backShots;
      ShotTable.Shot shot = table.lookup(distance.in(Feet));
      return manualDirection.setpoint(shot.hoodDeg(), shot.flywheelFps());
    }));
    joystick.rightTrigger().and(calibrating).whileTrue(new ShootCommand(shooter, hopper, calibration::setpoint));

    joystick.a().and(manual).onTrue(Commands.runOnce(() -> manualDirection = manualDirection.other()));
    joystick.a().and(calibrating).onTrue(Commands.runOnce(
        () -> calibration.toggleDirection(calibrationDistance().feet())));

    joystick.leftBumper().and(manual).onTrue(Commands.runOnce(() -> distance = distance.minus(Feet.of(1.0))));
    joystick.rightBumper().and(manual).onTrue(Commands.runOnce(() -> distance = distance.plus(Feet.of(1.0))));
    joystick.leftBumper().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepFlywheel(-2.0)));
    joystick.rightBumper().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepFlywheel(2.0)));

    joystick.povLeft().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepHood(-2.5)));
    joystick.povRight().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepHood(2.5)));
    joystick.povDown().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepHood(2.5)));

    joystick.y().and(calibrating).onTrue(Commands.runOnce(this::saveCalibrationShot));
```

The outside-calibration D-pad left/right/down presets keep their existing bodies, bound with `.and(manual)`.

Add methods:

```java
  private CalDistance calibrationDistance() {
    if (calibration.useManualDistance()) {
      return new CalDistance(calibration.manualDistanceFt(), "MANUAL");
    }
    var target = vision.getAimTarget(DriverStation.getAlliance());
    if (target.isEmpty()) {
      return new CalDistance(Double.NaN, "NONE");
    }
    double meters = HubAim.currentShotDistanceMeters(
        drivetrain.getRobotPose(), target.get(), calibration.direction().launchPoint());
    return new CalDistance(Units.metersToFeet(meters), "VISION");
  }

  private void saveCalibrationShot() {
    CalDistance d = calibrationDistance();
    boolean fresh = vision.isVisionFresh(AimConstants.kMaxVisionAgeToFeedSec);
    var refusal = ShotRecorder.refusalReason(d.feet(), calibration.useManualDistance(), fresh);
    if (refusal.isPresent()) {
      calibrationDashboard.setLastSaved("NOT SAVED: " + refusal.get());
      return;
    }
    ShotSetpoint sp = calibration.setpoint();
    calibrationDashboard.setLastSaved(shotRecorder.record(new ShotRecorder.Row(
        Timer.getFPGATimestamp(), calibration.direction(), d.feet(), d.source(),
        sp.hoodDeg(), shooter.getHoodAngle().in(Degrees),
        sp.flywheelFps(), shooter.getFlywheelVelocity().in(FeetPerSecond), fresh)));
  }

  /** Called from Robot.robotPeriodic(). */
  public void periodic() {
    CalDistance d = calibrationDistance();
    calibrationDashboard.update(d.feet(), d.source(),
        vision.isVisionFresh(AimConstants.kMaxVisionAgeToFeedSec),
        shooter.getHoodAngle().in(Degrees), shooter.getFlywheelVelocity().in(FeetPerSecond));
    SmartDashboard.putString("Mode", mode.toString());
    SmartDashboard.putString("ManualDirection", manualDirection.name());
    SmartDashboard.putNumber("Target Shooting Distance (Ft)", distance.in(Feet));
  }
```

In `Robot.robotPeriodic()` replace the commented shooter/vision calls with `m_robotContainer.periodic();`.

- [ ] **Step 7: Build and test** — `./gradlew build --offline` → BUILD SUCCESSFUL, all tests pass.

---

### Task 9: Simulation smoke test and documentation

**Files:**
- Modify: `docs/CODEMAP.md`, `docs/shooter-vision-guide.html` (republish to https://claude.ai/artifact/Q3wnhDighJcLqfQqoD97q9), `TODO.md`, shared checklist (https://claude.ai/artifact/NZZztxECjGkLhqBKLPhdW3, collection `tasks`)

- [ ] **Step 1: Simulation smoke test**

Run: `./gradlew simulateJava` in the background for ~20 s, then stop it.
Expected: robot code starts with no exception stack traces; "Vision: 2 camera(s)" startup line printed.

- [ ] **Step 2: Update CODEMAP** — add the new packages (`targeting`, `calibration`, `util`), the new subsystems and commands, remove `PointToHub`/`ShooterCalculator`, replace the controller table with spec section 5, remove fixed issues from "Things to know".

- [ ] **Step 3: Update the guide** — rename "Controller buttons (current code)" to the new layout (controller picture + table per spec section 5), add a "Building the shot table" section (calibration workflow, `Cal/*` entries, CSV path, paste lines), add troubleshooting rows for `Shooter/BlockedBy` values and `Aim/Direction = NO_ALLIANCE`. Republish.

- [ ] **Step 4: Update TODO and checklist** — mark done: auto chooser items unaffected; "Restore shooter updates?" (replaced by commands), "Check the PointToHub constructor" (PointToHub deleted), "Fix PointToHub choosing the alliance only at boot" (fixed). Add: "Bring up vision in the lab (guide lab steps 1–6, then paste camera/launch/hood values into Constants)".
