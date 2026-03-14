package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Command for turning robot 180 degrees in place.
 */
public class TurnAround extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private Rotation2d targetRotation;

  private CommandXboxController controller;

  public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top

  public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private int controlsInverted;

  public TurnAround(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, int controlsInverted) {
    this.controller = controller;
    this.drivetrain = drivetrain;
    this.controlsInverted = controlsInverted;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();
    this.targetRotation = currentRotation.plus(Rotation2d.fromDegrees(180));
  }

  @Override
  public void execute() {
    SwerveRequest drive = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025)
        .withTargetDirection(targetRotation)
        .withVelocityX(-controller.getLeftY() * MaxSpeed * controlsInverted)
        .withVelocityY(-controller.getLeftX() * MaxSpeed * controlsInverted);

    this.drivetrain.setControl(drive);
  }

  @Override
  public boolean isFinished() {
    Rotation2d currentRot = drivetrain.getState().Pose.getRotation();
    double angleDifference = currentRot.getDegrees() - targetRotation.getDegrees();
    return (Math.abs(angleDifference) <= 5);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drivetrain.setControl(new SwerveRequest.FieldCentric().withRotationalRate(0));
    }

    super.end(interrupted);
  }

}
