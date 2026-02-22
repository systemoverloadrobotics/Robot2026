// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.ShooterCalculator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // joystick.y().whileTrue(Commands.run(
    // () -> intakeSubsystem.setPower(Constants.Intake.OuttakePower),
    // intakeSubsystem))
    // .onFalse(Commands.runOnce(() -> intakeSubsystem.stop(), intakeSubsystem));

    joystick.y().onFalse(Commands.runOnce(() -> {
      if (mode == Mode.MANUAL) {
        mode = Mode.AUTO;
      } else {
        mode = Mode.MANUAL;
      }
    }, shooter, hopper));

    joystick.povDown().onFalse(Commands.runOnce(() -> {
      mode = Mode.CALIBRATION;
    }, shooter, hopper));

    joystick.leftTrigger().whileTrue(
        Commands.runOnce(() -> {
          isShooting = true;
        }, shooter).onlyIf(() -> mode == Mode.MANUAL)).onFalse(Commands.runOnce(() -> {
          isShooting = false;
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));

    joystick.leftTrigger().whileTrue(
        Commands.runOnce(() -> {
          shooter.setFlywheelVelocity(FeetPerSecond.of(targetFlywheelVelocity));
          shooter.setHoodAngle(Degrees.of(targetHoodAngle));
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION)).onFalse(Commands.runOnce(() -> {
          shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION));

    joystick.rightTrigger().whileTrue(
        Commands.runOnce(() -> {
          intakeSubsystem.setPivotPosition(Degrees.of(0.0));
          intakeSubsystem.start();
        }, shooter).onlyIf(() -> mode == Mode.MANUAL || mode == Mode.CALIBRATION)).onFalse(Commands.runOnce(() -> {
          intakeSubsystem.stop();
        }, shooter).onlyIf(() -> mode == Mode.MANUAL || mode == Mode.CALIBRATION));

    joystick.leftBumper().onTrue(
        Commands.runOnce(() -> {
          distance = distance.minus(Feet.of(1.0));
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));
    joystick.rightBumper().onTrue(
        Commands.runOnce(() -> {
          distance = distance.plus(Feet.of(1.0));
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));

    joystick.leftBumper().onTrue(
        Commands.runOnce(() -> {
          targetFlywheelVelocity -= 2.0;
        }, shooter).onlyIf(() -> mode == Mode.CALIBRATION));
    joystick.rightBumper().onTrue(
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
  }

  public void updateShooter() {
    if (mode != Mode.AUTO && mode != Mode.MANUAL) {
      return; // Only update shooter in AUTO or MANUAL mode
    }
    var flywheelSpeed = ShooterCalculator.getRegressionVelocity(distance);
    var launchAngle = ShooterCalculator.getRegressionAngle(distance);

    var launchAngleAdjusted = Degrees.of(-1 * launchAngle.in(Degrees) * side.getDirection());
    if (isShooting) {
      shooter.setHoodAngle(launchAngleAdjusted);
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

    if (shooter.isAtTarget() && isShooting) {
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
