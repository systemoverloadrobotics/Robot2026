package frc.robot.commands;

import static edu.wpi.first.units.Units.FeetPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShotConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.targeting.TagSpinDirection;

/**
 * Home test (simulator only): while held, spin the flywheel the way the camera that sees a tag
 * says - front camera = FRONT (+), back camera = BACK (-) - at a low test speed. Stops when no
 * tag (or both cameras) is seen. Never moves the hood and never feeds.
 */
public class TagSpinTestCommand extends Command {
  private final VisionSubsystem m_vision;
  private final ShooterSubSystem m_shooter;

  public TagSpinTestCommand(VisionSubsystem vision, ShooterSubSystem shooter) {
    m_vision = vision;
    m_shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    boolean front = m_vision.cameraSeesTag(VisionConstants.kFrontCamera.name());
    boolean back = m_vision.cameraSeesTag(VisionConstants.kBackCamera.name());
    TagSpinDirection.Choice choice = TagSpinDirection.choose(front, back);
    double fps = choice.flywheelFps(ShotConstants.kTagSpinTestFps);
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(fps));
    DogLog.log("TagSpin/FrontSeesTag", front);
    DogLog.log("TagSpin/BackSeesTag", back);
    DogLog.log("TagSpin/Direction", choice.label());
    DogLog.log("TagSpin/TargetFps", fps);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
    DogLog.log("TagSpin/Direction", "OFF");
    DogLog.log("TagSpin/TargetFps", 0.0);
  }
}
