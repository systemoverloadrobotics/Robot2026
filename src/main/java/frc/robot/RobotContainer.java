// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.RollerState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.ShooterCalculator;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubSystem shooter = new ShooterSubSystem();
  private final Storage storage = new Storage();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private double targetFlywheelVelocity = 50.0; // ft/s

  private Mode mode = Mode.MANUAL;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController joystick = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    joystick.y().whileTrue(Commands.run(
        () -> intakeSubsystem.setPower(Constants.Intake.OuttakePower), intakeSubsystem))
        .onFalse(Commands.runOnce(() -> intakeSubsystem.stop(), intakeSubsystem));

    joystick.leftTrigger().onTrue(
        Commands.runOnce(() -> {

          shooter.setFlywheelVelocity(FeetPerSecond.of(targetFlywheelVelocity));

        }, shooter).onlyIf(() -> mode == Mode.MANUAL)).onFalse(Commands.runOnce(() -> {
          shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));

    joystick.leftBumper().onTrue(
        Commands.runOnce(() -> {
          targetFlywheelVelocity -= 2.0;
          shooter.setFlywheelVelocity(FeetPerSecond.of(targetFlywheelVelocity));
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));
    joystick.rightBumper().onTrue(
        Commands.runOnce(() -> {
          targetFlywheelVelocity += 2.0;
          shooter.setFlywheelVelocity(FeetPerSecond.of(targetFlywheelVelocity));
        }, shooter).onlyIf(() -> mode == Mode.MANUAL));
  }

  public Command shootFuel(Distance distance, Side side) {

    var flywheelSpeed = ShooterCalculator.getRegressionVelocity(distance);
    var launchAngle = ShooterCalculator.getRegressionAngle(distance);

    var launchAngleAdjusted = Degrees.of(-1 * launchAngle.in(Degrees) * side.getDirection());

    return Commands.run(() -> {
      shooter.setHoodAngle(launchAngleAdjusted);
      shooter.setFlywheelVelocity(flywheelSpeed);
    }, shooter)
    .andThen(
      Commands.runEnd(
        () -> storage.setRollers(RollerState.FORWARD), () -> storage.setRollers(RollerState.OFF), storage)
        .onlyWhile(() -> shooter.isFlywheelAtTarget() && shooter.isHoodAngleAtTarget())
    );
  }

  private void shuttleFuel() {
    storage.setRollers(RollerState.REVERSE); // ← Add this line
    intakeSubsystem.setPower(Constants.Intake.OuttakePower);
  }

  private void stopShuttle() {
    storage.setRollers(RollerState.OFF); // ← Add this to stop storage
    intakeSubsystem.stop();
  }

  public enum Mode {
    MANUAL, AUTO
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
