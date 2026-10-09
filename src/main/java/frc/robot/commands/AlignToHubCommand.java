package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.targeting.HubAim;
import frc.robot.targeting.ShotDirection;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Hold X: turn whichever end of the robot needs less rotation toward the hub while the driver
 * keeps translating with the left stick. Aims from the fused pose, never raw camera bearings.
 * The direction is chosen once at press time so it can't flip mid-turn.
 */
public class AlignToHubCommand extends Command {
  protected final VisionSwerveDrivetrain m_drivetrain;
  protected final VisionSubsystem m_vision;
  private final DoubleSupplier m_vxOperator;
  private final DoubleSupplier m_vyOperator;

  protected Optional<Translation2d> m_target = Optional.empty();
  protected ShotDirection m_direction = ShotDirection.FRONT;
  protected HubAim.Solution m_solution;
  private boolean m_red;

  public AlignToHubCommand(
      VisionSwerveDrivetrain drivetrain,
      VisionSubsystem vision,
      DoubleSupplier vxOperatorMps,
      DoubleSupplier vyOperatorMps) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_vxOperator = vxOperatorMps;
    m_vyOperator = vyOperatorMps;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    m_red = alliance.orElse(Alliance.Blue) == Alliance.Red;
    m_target = m_vision.getAimTarget(alliance);
    m_solution = null;
    if (m_target.isPresent()) {
      m_direction = HubAim.chooseDirection(
          m_drivetrain.getRobotPose(), m_target.get(),
          ShotDirection.FRONT.launchPoint(), ShotDirection.BACK.launchPoint());
      DogLog.log("Aim/Direction", m_direction.name());
    } else {
      DogLog.log("Aim/Direction", alliance.isEmpty() ? "NO_ALLIANCE" : "NO_TARGET");
    }
  }

  @Override
  public void execute() {
    Translation2d field = HubAim.operatorToField(
        m_vxOperator.getAsDouble(), m_vyOperator.getAsDouble(), m_red);
    if (m_target.isEmpty()) {
      m_solution = null;
      m_drivetrain.driveFieldRelative(field.getX(), field.getY(), 0.0);
      return;
    }
    Pose2d pose = m_drivetrain.getRobotPose();
    m_solution = HubAim.solve(pose, m_target.get(), m_direction, m_direction.launchPoint());
    m_drivetrain.driveFacingAngle(field.getX(), field.getY(), m_solution.robotHeading());

    double errorDeg = m_solution.headingError().getDegrees();
    DogLog.log("Aim/HeadingErrorDeg", errorDeg);
    DogLog.log("Aim/DistanceFt", Units.metersToFeet(m_solution.shotDistanceMeters()));
    DogLog.log("Aim/Aimed", Math.abs(errorDeg) <= AimConstants.kAimToleranceDeg);
    DogLog.log("Aim/VisionFresh", m_vision.isVisionFresh(AimConstants.kMaxVisionAgeToFeedSec));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
  }
}
