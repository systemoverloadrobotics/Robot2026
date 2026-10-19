// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;



import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
    public static class Intake {
      public static final double KP = 25.0;
      //public static final double KPPushUp = 20.0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final int ROLLER_ID = 32;
      public static final int PIVOT_ID = 31;
      public static final int ENCODER_ID = 33; 
      public static final int PivotPID = 0; 
      public static final double PivotSensorToMechanism = 26.66666667;
      public static final double StartPower = 0.90;
      public static final double PushUpPower = -0.1;
      public static final double StopPower = 0; 
      public static final double OuttakePower = -0.5; 
      // TEST ONLY: scales every intake roller power while the new 46:24 pulley is tested.
      // Set back to 1.0 before competition (see TODO.md).
      public static final double RollerPowerScale = 0.75;
      // Caps pivot motor voltage, which sets how fast the arm swings (was 12 V).
      public static final double PivotMaxVolts = 10.0;
      public static final Angle IntakePosition = Degrees.of(105); // arm hard stop is ~110 deg
      public static final Angle IntakeStowedPosition = Degrees.of(-6);
      public static final Angle IntakePushUp = Degrees.of(40);
      public static final Angle IntakePushDown = Degrees.of(90);
      public static final Angle IntakeError = Degrees.of(10);
      public static final Angle PivotOffset = Rotation.of(-0.471);
    }
    
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

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
    /** Tag 100 in practice-layout.json: a virtual tag (never printed) at the lab hub's center. */
    public static final int kLabTargetTagId = 100;
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

    // PLACEHOLDER names and positions until lab steps 3 and 4 (see vision-docs-2026/docs/5_guide_arcimedes-shooter-and-vision.html).
    // Front camera: front-right corner under the hood, facing forward.
    public static final CameraConfig kFrontCamera = new CameraConfig(
        "FrontCamera",
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.0), Units.inchesToMeters(-11.0), Units.inchesToMeters(15.4)),
            new Rotation3d(0.0, Math.toRadians(-30.0), 0.0)),
        true);
    // Back camera: back-right corner, facing backward (yaw 180 degrees).
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
    /** Consistent jump-rejected estimates in a row before the pose snaps to them. */
    public static final int kJumpRecoveryFrames = 10;
    /** How close those estimates must be to each other (m). */
    public static final double kJumpRecoveryConsistencyMeters = 0.3;
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
    /** Start-button reset only uses a vision pose newer than this (an old pose lags the robot). */
    public static final double kMaxVisionAgeToResetSec = 0.2;
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
    /** Never feed below this commanded flywheel speed (ft/s). */
    public static final double kMinFeedFlywheelFps = 5.0;
    /** Tag spin test (simulator only, hold Y in MANUAL): flywheel speed magnitude, ft/s. */
    public static final double kTagSpinTestFps = 10.0;
    /** Flywheel moment of inertia for the simulator, kg*m^2. A guess; only affects spin-up time. */
    public static final double kSimFlywheelMoiKgM2 = 0.004;
    // Shot table rows {distance ft, hood deg magnitude, flywheel ft/s magnitude}.
    // UNVERIFIED SEED from the old ShooterCalculator (removed in 1ef12b3): its flywheel
    // regression (rot/s, 4 in wheel 1:1 -> x1.047 ft/s) at a fixed 2 deg hood. That hood angle
    // conflicts with the trench presets (-88/+87), so check every row in CALIBRATION mode and
    // replace it with saved rows (Y). Past ~20 ft the old fit falls apart, so it stops there.
    public static final double[][] kFrontShots = {
        {4.0, 2.0, 19.7}, {6.0, 2.0, 20.6}, {8.0, 2.0, 21.3}, {10.0, 2.0, 22.3}, {12.0, 2.0, 23.8},
        {14.0, 2.0, 25.8}, {16.0, 2.0, 28.0}, {18.0, 2.0, 29.8}, {20.0, 2.0, 30.3}};
    public static final double[][] kBackShots = {
        {4.0, 2.0, 19.7}, {6.0, 2.0, 20.6}, {8.0, 2.0, 21.3}, {10.0, 2.0, 22.3}, {12.0, 2.0, 23.8},
        {14.0, 2.0, 25.8}, {16.0, 2.0, 28.0}, {18.0, 2.0, 29.8}, {20.0, 2.0, 30.3}};
  }

  public static class Climb {
    public static final String CANBUS_RIO = "rio";
    public static final int MOTOR_ID = -1;
    public static final double KP = -1;
    public static final double KI = -1;
    public static final double KD = -1;
    public static final double SENSOR_TO_MECHANISM_RATIO = -1;
    public static final double SUPPLY_CURRENT_LIMIT = -1;
    public static final double MAX_EXTN_POSITION = -1;
    public static final double MIN_EXTN_POSITION = -1;
    public static final double HOLD_POSITION = -1;
    public static final double POSITION_TOLERANCE = 0.1; // rotations (placeholder)
  }

  public static class Hopper {
    public static final int ROLLER_MOTOR_ID = 21;
    public static final int BOTTOM_ROLLER_MOTOR_ID = 23; 
    public static final double ROLLER_FORWARD_SPEED = 1.0;
    public static final double ROLLER_REVERSE_SPEED = -0.5;
    public static final int SPINDEXER_ID = 22; // todo: change value
    public static final double SPINDEXER_RATIO = 1.0; // todo: change value, ratio of speed of spindexer to speed of intake rollers
  }

  public static class Shooter {
    public static final int TOP_FLYWHEEL_ID = 13;
    public static final int BOTTOM_FLYWHEEL_ID = 14;
    public static final int SHOOTER_PIVOT_ID = 11;

    public static final int SHOOTER_PIVOT_ENCODER = 18;

    public static final double FLYWHEEL_kP = 0.12448;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.000;
    public static final double FLYWHEEL_kV = 0.12;

    public static final double HOOD_ANGLE_KP = 20.0;
    public static final double HOOD_ANGLE_KI = 0.0;
    public static final double HOOD_ANGLE_KD = 0.000;

    public static final double FLYWHEEL_DIAMETER_INCHES = 4.0;
    public static final double FLYWHEEL_CIRCUMFERENCE_FT = Math.PI * FLYWHEEL_DIAMETER_INCHES / 12.0;

    public static final double FLYWHEEL_GEAR_RATIO = 1.0;
    public static final double FLYWHEEL_FPS_TOLERANCE = 5.0;
    public static final double SHOOTER_PIVOT_TOLERANCE = 6.0; // was 4.0
    public static final double SHOOTER_PIVOT_GEAR_RATIO = 16.457;

    public static final double LEFT_HOOD_MIN_ANGLE = 15.0;
    public static final double LEFT_HOOD_MAX_ANGLE = 45.0;
    public static final double LEFT_HOOD_BASE_ANGLE = 30.0;  // Center of left range

    public static final double RIGHT_HOOD_MIN_ANGLE = 135.0;
    public static final double RIGHT_HOOD_MAX_ANGLE = 165.0;
    public static final double RIGHT_HOOD_BASE_ANGLE = 150.0;  // Center of right range

    public static final double HOOD_ABSOLUTE_MIN = 10.0;
    public static final double HOOD_ABSOLUTE_MAX = 170.0;

    // PRESET SHOOTING POSITIONS

    // public static final double LEFT_TRENCH_HOOD_ANGLE = -75;
    public static final double LEFT_TRENCH_HOOD_ANGLE = -88; // based on shot data
    public static final double LEFT_TRENCH_FLYWHEEL_FPS = 42;
    // public static final double LEFT_TRENCH_FLYWHEEL_FPS = -22.12879;
    // public static final double LEFT_TRENCH_FLYWHEEL_FPS = -40;

    // public static final double RIGHT_TRENCH_HOOD_ANGLE = 75;
    public static final double RIGHT_TRENCH_HOOD_ANGLE = 87;
    // public static final double RIGHT_TRENCH_FLYWHEEL_FPS = 32.56;
    // public static final double LEFT_TRENCH_FLYWHEEL_FPS = 22.12879;
    public static final double RIGHT_TRENCH_FLYWHEEL_FPS = -42;
  }
}
