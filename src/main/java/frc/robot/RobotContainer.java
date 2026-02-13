// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.RollerState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


import static edu.wpi.first.units.Units.Degree;
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
    joystick.b().whileTrue(Commands.runOnce(
      () -> shooter.setFlywheelVelocity(FeetPerSecond.of(targetFlywheelVelocity)), shooter))
      .onFalse(Commands.runOnce(() -> shooter.setFlywheelVelocity(FeetPerSecond.of(0.0))))
      ;

      joystick.a().onTrue(Commands.runOnce(
        () -> targetFlywheelVelocity += 1.0, (Subsystem)null));


      joystick.x().onTrue(Commands.runOnce(
        () -> targetFlywheelVelocity -= 1.0, (Subsystem)null));
        
      
      joystick.y().whileTrue(Commands.run(
      () -> intakeSubsystem.setPower(Constants.Intake.OuttakePower), intakeSubsystem))
      .onFalse(Commands.runOnce(() -> intakeSubsystem.stop(), intakeSubsystem));

  }

  public Command shootFuel(Distance distance) {

    var flywheelSpeed = getFlywheelSpeed(distance);
    var launchAngle = getLaunchAngle(distance);

    return Commands.run(() -> {
      shooter.setHoodAngle(launchAngle);
      shooter.setFlywheelVelocity(flywheelSpeed);
    }, shooter)
        .andThen(Commands.waitUntil(
            () -> shooter.isFlywheelAtTarget() && shooter.isHoodAngleAtTarget()))
        .andThen(Commands.runEnd(
            () -> storage.setRollers(RollerState.FORWARD),
            () -> {
              this.stopFuelShooter();
              storage.setRollers(RollerState.OFF);
            },
            storage));
  }
  private void shuttleFuel() {
    storage.setRollers(RollerState.REVERSE);  // ← Add this line
    intakeSubsystem.setPower(Constants.Intake.OuttakePower);
}

private void stopShuttle() {
    storage.setRollers(RollerState.OFF);  // ← Add this to stop storage
    intakeSubsystem.stop();
}



  // TODO - Stop Shooting Command
  public void stopFuelShooter() {
    // m_storage.setRollers(RollerState.OFF);
    // stop FlyWheel
    // set Hood to default posiiton
  }

  // TODO: Add function for LaunchAngle
  public Angle getLaunchAngle(Distance distance) {
    return Degree.of(0.0);
  }

  // TODO: Add function for Flywheel speed
  public LinearVelocity getFlywheelSpeed(Distance distance) {
    return FeetPerSecond.of(0.0);
  }
}
