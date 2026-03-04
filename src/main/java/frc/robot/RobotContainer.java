// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Intake;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PointToHub;
import frc.robot.commands.PointToHub.Alignment;
import frc.robot.commands.PointToHub.Strategy;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.ShooterCalculator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubSystem shooter = new ShooterSubSystem();
  private final Hopper hopper = new Hopper();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final Timer matchTimer = new Timer();

  private double targetFlywheelVelocity = 50.0; // ft/s

  private double targetHoodAngle = 0.0; // degrees

  private Mode mode = Mode.MANUAL;

  private Distance distance = Feet.of(8.0);
  private Side side = Side.LEFT;

  private boolean isShooting = false;

  private final CommandXboxController joystick = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public final PointToHub pointToHub = new PointToHub(drivetrain, joystick, Alignment.LEFT, Strategy.SINGLE_TAG);

  public int controlsInverted = 1;

  public RobotContainer() {
    NamedCommands.registerCommand("intakeDown", Commands.runOnce(() -> {
      intakeSubsystem.setPivotPosition(Intake.IntakePosition);
      if (intakeSubsystem.atIntake()) {
        intakeSubsystem.setPower(-0.5);
      } else {
        intakeSubsystem.setPower(0.6);
      }
    }, shooter));

    NamedCommands.registerCommand("intakeStop", Commands.runOnce(() -> {
      intakeSubsystem.stop();
    }));

    configureBindings();
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
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * controlsInverted) // Drive forward
                                                                                                      // with
            // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed * controlsInverted) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.b().whileTrue(pointToHub.onlyIf(() -> mode == Mode.AUTO));
    joystick.a().onFalse(Commands.runOnce(() -> pointToHub.resetTranslationPoseWithVision(), drivetrain)
        .onlyIf(() -> mode == Mode.AUTO));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.start().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.x().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX())))
        .onlyIf(() -> mode == Mode.MANUAL));

    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.y().onFalse(Commands.runOnce(() -> {
      if (mode == Mode.MANUAL) {
        mode = Mode.AUTO;
      } else {
        mode = Mode.MANUAL;
      }
    }, shooter, hopper));

    joystick.a().onFalse(Commands.runOnce(() -> {
      if (side == Side.LEFT) {
        side = Side.RIGHT;
      } else {
        side = Side.LEFT;
      }
    }));

    joystick.povDown().onFalse(Commands.runOnce(() -> {
      mode = Mode.CALIBRATION;
    }, shooter, hopper));

    joystick.rightTrigger().whileTrue(
        Commands.runOnce(() -> {
          isShooting = true;
        }, shooter).onlyIf(() -> mode == Mode.MANUAL)).onFalse(Commands.runOnce(() -> {
          isShooting = false;
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));

    joystick.rightTrigger().whileTrue(
        Commands.runOnce(() -> {
          shooter.setFlywheelVelocity(FeetPerSecond.of(targetFlywheelVelocity));
          shooter.setHoodAngle(Degrees.of(targetHoodAngle));
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION)).onFalse(Commands.runOnce(() -> {
          shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION));

    joystick.leftTrigger().whileTrue(
        Commands.runOnce(() -> {
          intakeSubsystem.setPivotPosition(Intake.IntakePosition);
          if (intakeSubsystem.atIntake()) {
            intakeSubsystem.setPower(-0.5);
          } else {
            intakeSubsystem.setPower(0.6);
          }
        }, shooter).onlyIf(() -> mode == Mode.MANUAL || mode == Mode.CALIBRATION)).onFalse(Commands.runOnce(() -> {
          // intakeSubsystem.setPivotPosition(Degrees.of(0));
          intakeSubsystem.stop();
        }, shooter).onlyIf(() -> mode == Mode.MANUAL || mode == Mode.CALIBRATION));

    joystick.leftBumper().onTrue( // was assigned to rightBumper
        Commands.runOnce(() -> {
          distance = distance.minus(Feet.of(1.0));
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));
    joystick.rightBumper().onTrue( // was assigned to leftBumper
        Commands.runOnce(() -> {
          distance = distance.plus(Feet.of(1.0));
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));

    joystick.rightBumper().onTrue( // was assigned to leftBumper
        Commands.runOnce(() -> {
          targetFlywheelVelocity -= 2.0;
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION));
    joystick.leftBumper().onTrue( // was assigned to rightBumper
        Commands.runOnce(() -> {
          targetFlywheelVelocity += 2.0;
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION));

    joystick.povLeft().onTrue(
        Commands.runOnce(() -> {
          targetHoodAngle -= 2.5;
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION));
    joystick.povRight().onTrue(
        Commands.runOnce(() -> {
          targetHoodAngle += 2.5;
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION));

    joystick.back().debounce(0.02).onTrue(Commands.runOnce(() -> {
      controlsInverted = -controlsInverted;
    }, drivetrain));
  }

  public void updateShooter() {
    SmartDashboard.putNumber("Target Shooting Distance", distance.in(Feet));

    if (mode != Mode.AUTO && mode != Mode.MANUAL) {
      return; // Only update shooter in AUTO or MANUAL mode
    }
    var flywheelSpeed = ShooterCalculator.getRegressionVelocity(distance);
    var launchAngle = ShooterCalculator.getRegressionAngle(distance);

    var launchAngleAdjusted = Degrees.of(-1 * launchAngle.in(Degrees) * side.getDirection());

    // For right side
    if (launchAngleAdjusted.in(Degrees) < 0) {
      flywheelSpeed = flywheelSpeed.times(-1);
    }

    // shooter.setHoodAngle(launchAngleAdjusted);

    if (isShooting) {
      shooter.setFlywheelVelocity(flywheelSpeed);
    } else {
      shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
    }

    /*
     * Three Conditions to feed to shooter:
     * 1. Is flywheel at target velocity?
     * 2. Is hood at target angle?
     * 3. Is drivetrain aligned to hub?
     * 4. Is vision measurement accurate?
     * 5. Is it our turn to shoot?
     */

    // if (shooter.isAtTarget() && isShooting) {
    if (shooter.isFlywheelAtTarget() && isShooting) {
      hopper.setRollers(RollerState.FORWARD);
    } else {
      hopper.setRollers(RollerState.OFF);
    }
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
    hopper.setRollers(RollerState.REVERSE); // ← Add this line
    intakeSubsystem.setPower(Constants.Intake.OuttakePower);
  }

  private void stopShuttle() {
    hopper.setRollers(RollerState.OFF); // ← Add this to stop storage
    intakeSubsystem.stop();
  }

  public enum Mode {
    MANUAL, AUTO, CALIBRATION
  }

  public enum Side {
    LEFT(-1), RIGHT(1);

    private final int direction;

    private Side(int direction) {
      this.direction = direction;
    }

    public int getDirection() {
      return direction;
    }
  }
}
