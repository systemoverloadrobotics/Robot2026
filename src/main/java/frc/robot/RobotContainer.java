// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AimConstants;
import frc.robot.Constants.Intake;
import frc.robot.commands.AlignAndShootCommand;
import frc.robot.commands.AlignToHubCommand;
import frc.robot.calibration.CalibrationDashboard;
import frc.robot.calibration.CalibrationState;
import frc.robot.calibration.ShotRecorder;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TagSpinTestCommand;
import frc.robot.targeting.HubAim;
import frc.robot.targeting.ShotDirection;
import frc.robot.targeting.ShotSetpoint;
import frc.robot.targeting.ShotTable;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import dev.doglog.DogLog;
import frc.robot.dashboard.DashboardPublisher;
import frc.robot.dashboard.NamedDevice;
import java.util.ArrayList;
import java.util.List;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import frc.robot.util.StickShaping;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Shooter.LEFT_TRENCH_HOOD_ANGLE;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubSystem shooter = new ShooterSubSystem();
  private final Hopper hopper = new Hopper();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final Timer matchTimer = new Timer();

  private Mode mode = Mode.MANUAL;

  private boolean intakeRunning = false;

  private Distance distance = Feet.of(4.0);
  private ShotDirection manualDirection = ShotDirection.FRONT;

  private final CommandXboxController joystick = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private double MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final VisionSwerveDrivetrain drivetrain = new VisionSwerveDrivetrain();
  private final VisionSubsystem vision = new VisionSubsystem(drivetrain);

  private final ShotTable frontShots = ShotTable.forDirection(ShotDirection.FRONT);
  private final ShotTable backShots = ShotTable.forDirection(ShotDirection.BACK);

  private final CalibrationState calibration = new CalibrationState(frontShots, backShots);
  private final CalibrationDashboard calibrationDashboard = new CalibrationDashboard(calibration);
  private final ShotRecorder shotRecorder = new ShotRecorder(
      Filesystem.getOperatingDirectory().toPath().resolve("logs").resolve("shot-calibration.csv"));

  /** PDH (or PDP) at its default CAN ID; also gives DogLog every channel's current. */
  private final PowerDistribution pdh = new PowerDistribution();
  private final DashboardPublisher dashboard;

  /** Dashboard edits are ignored until the first enable (stale values after a restart). */
  private boolean enabledOnce = false;

  /** Distance used by calibration, and where it came from (VISION, MANUAL or NONE). */
  private record CalDistance(double feet, String source) {}

  public int controlsInverted = 1;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    DogLog.setPdh(pdh);
    dashboard = new DashboardPublisher(vision, pdh, mechanismDevices(), driveDevices(),
        () -> mode.toString(),
        () -> (mode == Mode.CALIBRATION ? calibration.direction() : manualDirection).name());

    NamedCommands.registerCommand("intakeDown", Commands.runOnce(() -> {
      intakeSubsystem.setPivotPosition(Intake.IntakePosition);
      intakeSubsystem.setPower(-0.8);
    }, intakeSubsystem));

    NamedCommands.registerCommand("intakeStart", Commands.runOnce(() -> {
      intakeSubsystem.setPower(-0.8);
    }, intakeSubsystem));

    NamedCommands.registerCommand("intakeStop", Commands.runOnce(() -> {
      intakeSubsystem.stop();
    }, intakeSubsystem));

    NamedCommands.registerCommand("shootRight",
        shootFromTrench(Constants.Shooter.RIGHT_TRENCH_HOOD_ANGLE, Constants.Shooter.RIGHT_TRENCH_FLYWHEEL_FPS));

    NamedCommands.registerCommand("shootPreloadLeft",
        shootFromTrench(Constants.Shooter.LEFT_TRENCH_HOOD_ANGLE, Constants.Shooter.LEFT_TRENCH_FLYWHEEL_FPS));

    NamedCommands.registerCommand("stopShoot", Commands.runOnce(() -> {
      shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
      shooter.idleHood();
      hopper.setRollers(RollerState.OFF);
    }, shooter, hopper));

    AlignAndShootCommand autoShot = new AlignAndShootCommand(
        drivetrain, vision, shooter, hopper, frontShots, backShots, () -> 0.0, () -> 0.0);
    NamedCommands.registerCommand("alignAndShoot",
        autoShot.until(() -> autoShot.getSecondsFed() >= AimConstants.kAutoFeedSeconds)
            .withTimeout(AimConstants.kAutoTimeoutSeconds));

    configureBindings();

    drivetrain.configAuto();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // joystick.y().whileTrue(Commands.run(
    // () -> intakeSubsystem.setPower(Constants.Intake.OuttakePower),
    // intakeSubsystem))
    // .onFalse(Commands.runOnce(() -> intakeSubsystem.stop(), intakeSubsystem));

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-StickShaping.square(joystick.getLeftY()) * MaxSpeed * controlsInverted) // Drive
                                                                                                              // forward
            // with
            // negative Y (forward)
            .withVelocityY(-StickShaping.square(joystick.getLeftX()) * MaxSpeed * controlsInverted) // Drive left with negative X (left)
            .withRotationalRate(-StickShaping.square(joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // On disable, clear the intake's last request so enabling doesn't resume an old deploy,
    // and reset the toggle so the next left-trigger press deploys.
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
      intakeSubsystem.stopAll();
      intakeRunning = false;
    }, intakeSubsystem).ignoringDisable(true));

    joystick.back().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.x().whileTrue(new AlignToHubCommand(drivetrain, vision, this::stickVx, this::stickVy));
    joystick.b().whileTrue(new AlignAndShootCommand(
        drivetrain, vision, shooter, hopper, frontShots, backShots, this::stickVx, this::stickVy));

    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.start().onFalse(Commands.runOnce(vision::resetTranslationFromVision).ignoringDisable(true));

    Trigger manual = new Trigger(() -> mode == Mode.MANUAL);
    Trigger calibrating = new Trigger(() -> mode == Mode.CALIBRATION);

    // D-pad up: CALIBRATION on/off. Entering starts from the shot table, not 0 deg / 50 ft/s.
    joystick.povUp().onTrue(Commands.runOnce(() -> {
      if (mode == Mode.CALIBRATION) {
        mode = Mode.MANUAL;
      } else {
        mode = Mode.CALIBRATION;
        calibration.resetToTable(calibrationDistance().feet());
      }
    }));

    // Right trigger (hold): shoot without aiming. MANUAL uses the shot table at the bumper-set
    // distance; CALIBRATION uses the hand-set values. Both feed once the shooter is on target.
    joystick.rightTrigger().and(manual).whileTrue(new ShootCommand(shooter, hopper, () -> {
      ShotTable table = manualDirection == ShotDirection.FRONT ? frontShots : backShots;
      ShotTable.Shot shot = table.lookup(distance.in(Feet));
      return manualDirection.setpoint(shot.hoodDeg(), shot.flywheelFps());
    }));
    joystick.rightTrigger().and(calibrating).whileTrue(new ShootCommand(shooter, hopper, calibration::setpoint));

    joystick.leftTrigger().debounce(0.05).onTrue(
        Commands.runOnce(() -> {
          // DEBUG (intake not deploying): remove once fixed.
          System.out.println("[Intake] left trigger pressed: running=" + intakeRunning
              + " atIntake=" + intakeSubsystem.atIntake());
          DogLog.log("Intake/LeftTriggerPressed", true);
          if (intakeRunning) {
            intakeSubsystem.stop();
            intakeRunning = false;
          } else {
            if (!intakeSubsystem.atIntake()) {
              intakeSubsystem.setPivotPosition(Intake.IntakePosition);
            }
            intakeSubsystem.setPower(-0.8);
            intakeRunning = true;
          }
        }, intakeSubsystem));

    // Left stick push: stop the roller and stow the intake (from pinnacles 87d5ecb, which used D-pad up).
    joystick.leftStick().debounce(0.05).onTrue(
        Commands.runOnce(() -> {
          intakeSubsystem.stop();
          intakeRunning = false;
          intakeSubsystem.setPivotPosition(Intake.IntakeStowedPosition);
        }, intakeSubsystem));

    // A: shot direction FRONT / BACK (X and B choose their own direction).
    joystick.a().and(manual).onTrue(Commands.runOnce(() -> manualDirection = manualDirection.other()));
    joystick.a().and(calibrating).onTrue(Commands.runOnce(
        () -> calibration.toggleDirection(calibrationDistance().feet())));

    // Bumpers: MANUAL distance -1 / +1 ft; CALIBRATION flywheel -2 / +2 ft/s.
    joystick.leftBumper().and(manual).onTrue(Commands.runOnce(() -> distance = distance.minus(Feet.of(1.0))));
    joystick.rightBumper().and(manual).onTrue(Commands.runOnce(() -> distance = distance.plus(Feet.of(1.0))));
    joystick.leftBumper().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepFlywheel(-2.0)));
    joystick.rightBumper().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepFlywheel(2.0)));

    // D-pad left / right (hold): MANUAL shoots at the trench presets, feeding once the shooter is
    // on target (same as the right trigger); CALIBRATION hood -2.5 / +2.5 deg.
    joystick.povLeft().and(manual).whileTrue(new ShootCommand(shooter, hopper,
        () -> new ShotSetpoint(Constants.Shooter.LEFT_TRENCH_HOOD_ANGLE, Constants.Shooter.LEFT_TRENCH_FLYWHEEL_FPS)));
    joystick.povRight().and(manual).whileTrue(new ShootCommand(shooter, hopper,
        () -> new ShotSetpoint(Constants.Shooter.RIGHT_TRENCH_HOOD_ANGLE, Constants.Shooter.RIGHT_TRENCH_FLYWHEEL_FPS)));
    joystick.povLeft().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepHood(-2.5)));
    joystick.povRight().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepHood(2.5)));

    joystick.povDown().and(manual).onTrue(
        Commands.runOnce(() -> {
          // getDegrees() is in (-180, 180]: within 60 deg of 0 -> left trench, within 60 deg of 180 -> right.
          // Headings between 60 and 120 deg either way leave the shooter unchanged.
          double headingDeg = drivetrain.getState().Pose.getRotation().getDegrees();
          if (Math.abs(headingDeg) <= 60) {
          shooter.setHoodAngle(Degrees.of(Constants.Shooter.LEFT_TRENCH_HOOD_ANGLE));
          shooter.setFlywheelVelocity(FeetPerSecond.of(Constants.Shooter.LEFT_TRENCH_FLYWHEEL_FPS));
          } else if (Math.abs(headingDeg) >= 120) {
          shooter.setHoodAngle(Degrees.of(Constants.Shooter.RIGHT_TRENCH_HOOD_ANGLE));
          shooter.setFlywheelVelocity(FeetPerSecond.of(Constants.Shooter.RIGHT_TRENCH_FLYWHEEL_FPS));
          }
        }));
    joystick.povDown().and(calibrating).onTrue(Commands.runOnce(() -> calibration.stepHood(2.5)));

    // Y in CALIBRATION: save the current shot as a shot-table row.
    joystick.y().and(calibrating).onTrue(Commands.runOnce(this::saveCalibrationShot));
    // Home test, simulator only: spin the flywheel the way the camera that sees a tag says.
    joystick.y().and(manual).and(RobotBase::isSimulation)
        .whileTrue(new TagSpinTestCommand(vision, shooter));

    // No drivetrain requirement: it only flips a number, and requiring the drivetrain would
    // cancel the Back wheel lock and a held X or B.
    joystick.back().debounce(0.02).onTrue(Commands.runOnce(() -> {
      controlsInverted = -controlsInverted;
    }).onlyIf(() -> mode == Mode.MANUAL));
  }

  public void startMatchTimer() {
    matchTimer.reset();
    matchTimer.start();
  }

  public Command shootFuel() {
    return Commands.runEnd(
        () -> hopper.setRollers(RollerState.FORWARD), () -> hopper.setRollers(RollerState.OFF), hopper)
        .onlyWhile(() -> shooter.isFlywheelAtTarget() && shooter.isHoodAngleAtTarget());
  }

  private void shuttleFuel() {
    hopper.setRollers(RollerState.REVERSE); // add this line
    intakeSubsystem.setPower(Constants.Intake.OuttakePower);
  }

  private void stopShuttle() {
    hopper.setRollers(RollerState.OFF); // add this to stop storage
    intakeSubsystem.stop();
  }

  public enum Mode {
    MANUAL, CALIBRATION
  }

  /** Spins up the shooter to a trench preset, then feeds once it reaches target. */
  private Command shootFromTrench(double hoodAngleDegrees, double flywheelFps) {
    return new SequentialCommandGroup(
        Commands.runOnce(() -> {
          shooter.setHoodAngle(Degrees.of(hoodAngleDegrees));
          shooter.setFlywheelVelocity(FeetPerSecond.of(flywheelFps));
          intakeRunning = false;
        }, shooter),
        new WaitUntilCommand(() -> shooter.isAtTarget()),
        Commands.runOnce(() -> hopper.setRollers(RollerState.FORWARD), hopper));
  }

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
    if (DriverStation.isEnabled()) {
      enabledOnce = true;
    }
    CalDistance d = calibrationDistance();
    calibrationDashboard.update(d.feet(), d.source(),
        vision.isVisionFresh(AimConstants.kMaxVisionAgeToFeedSec),
        shooter.getHoodAngle().in(Degrees), shooter.getFlywheelVelocity().in(FeetPerSecond),
        enabledOnce);
    SmartDashboard.putString("Mode", mode.toString());
    // DEBUG (intake not deploying): remove once fixed.
    SmartDashboard.putNumber("Intake/LeftTriggerAxis", joystick.getLeftTriggerAxis());
    SmartDashboard.putBoolean("Intake/Running", intakeRunning);
    DogLog.log("Intake/LeftTriggerAxis", joystick.getLeftTriggerAxis());
    DogLog.log("Intake/Running", intakeRunning);
    SmartDashboard.putString("ManualDirection", manualDirection.name());
    SmartDashboard.putNumber("Target Shooting Distance (Ft)", distance.in(Feet));
    dashboard.periodic();
  }

  /** Every mechanism CAN device, for the Elastic dashboard. */
  private List<NamedDevice> mechanismDevices() {
    List<NamedDevice> devices = new ArrayList<>(shooter.getDevices());
    devices.addAll(hopper.getDevices());
    devices.addAll(intakeSubsystem.getDevices());
    return devices;
  }

  /** Swerve motors, encoders and the Pigeon, for the Elastic dashboard. */
  private List<NamedDevice> driveDevices() {
    List<NamedDevice> devices = new ArrayList<>();
    String[] names = {"FL", "FR", "BL", "BR"};
    var modules = drivetrain.getModules();
    for (int i = 0; i < modules.length; i++) {
      devices.add(NamedDevice.of(names[i] + " drive", modules[i].getDriveMotor()));
      devices.add(NamedDevice.of(names[i] + " steer", modules[i].getSteerMotor()));
      devices.add(NamedDevice.of(names[i] + " CANcoder", modules[i].getEncoder()));
    }
    devices.add(NamedDevice.of("Pigeon", drivetrain.getPigeon2()));
    return devices;
  }

  /** Driver left stick, operator-relative m/s, with the same squaring, sign and inversion as the default drive. */
  private double stickVx() {
    return -StickShaping.square(MathUtil.applyDeadband(joystick.getLeftY(), AimConstants.kStickDeadband)) * MaxSpeed * controlsInverted;
  }

  private double stickVy() {
    return -StickShaping.square(MathUtil.applyDeadband(joystick.getLeftX(), AimConstants.kStickDeadband)) * MaxSpeed * controlsInverted;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
