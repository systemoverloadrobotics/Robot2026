// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class IntakePushing extends Command {
  private final IntakeSubsystem m_intake;
  private int intakeTimer = 0;
  private boolean intakePushingUp = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public IntakePushing(IntakeSubsystem intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer = 0;
    intakePushingUp = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeTimer += 1;
    if (intakeTimer >= 50) {
        if (!intakePushingUp) {
          m_intake.setPivotPosition(Constants.Intake.IntakePushUp);
          intakePushingUp = true;
        } else {
          m_intake.setPivotPosition(Constants.Intake.IntakePushDown);
          intakePushingUp = false;
        }
        intakeTimer = 0;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setPivotPosition(Constants.Intake.IntakePosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}