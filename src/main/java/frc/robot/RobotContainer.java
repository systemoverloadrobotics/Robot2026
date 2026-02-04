// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.Storage.RollerState;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.sound.sampled.Line;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ShooterSubSystem m_shooterSubsystem = new ShooterSubSystem();
  private final Storage m_storage = new Storage();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public Command shootFuel(Distance distance) {

    var flywheelSpeed = getFlywheelSpeed(distance);
    var launchAngle = getLaunchAngle(distance);

    return Commands.run(() -> {
      m_shooterSubsystem.setHoodAngle(launchAngle);
      m_shooterSubsystem.setFlywheelVelocity(flywheelSpeed);
    }, m_shooterSubsystem)
        .andThen(Commands.waitUntil(
            () -> m_shooterSubsystem.isFlywheelAtTarget() && m_shooterSubsystem.isHoodAngleAtTarget()))
        .andThen(Commands.runEnd(
            () -> m_storage.setRollers(RollerState.FORWARD),
            () -> {
              this.stopFuelShooter();
              m_storage.setRollers(RollerState.OFF);
            },
            m_storage));
  }

  // TODO - Stop Shooting Command
  public void stopFuelShooter() {
    // m_storage.setRollers(RollerState.OFF);
    // stop FlyWheel
    // set Hood to default posiiton
  }

  // TODO: Add function for LaunchAngle
  public Angle getLaunchAngle(Distance distance) {
    return Angle.ofBaseUnits(0.0, Degree);
  }

  // TODO: Add function for Flywheel speed
  public LinearVelocity getFlywheelSpeed(Distance distance) {
    return FeetPerSecond.of(0.0);
  }
}
