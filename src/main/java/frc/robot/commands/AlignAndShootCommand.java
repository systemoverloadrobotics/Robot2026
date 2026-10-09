package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.targeting.ShotDirection;
import frc.robot.targeting.ShotGate;
import frc.robot.targeting.ShotSetpoint;
import frc.robot.targeting.ShotTable;
import frc.robot.util.SettleDetector;
import java.util.function.DoubleSupplier;

/**
 * Hold B: align (as AlignToHubCommand), set hood and flywheel from the shot table for the live
 * distance, and feed only when aimed, settled, stopped, on target and vision is fresh.
 */
public class AlignAndShootCommand extends AlignToHubCommand {
  private final ShooterSubSystem m_shooter;
  private final Hopper m_hopper;
  private final ShotTable m_frontTable;
  private final ShotTable m_backTable;
  private final SettleDetector m_settle = new SettleDetector(AimConstants.kSettledLoops);
  private double m_secondsFed;
  private double m_lastTimestamp;

  public AlignAndShootCommand(
      VisionSwerveDrivetrain drivetrain,
      VisionSubsystem vision,
      ShooterSubSystem shooter,
      Hopper hopper,
      ShotTable frontTable,
      ShotTable backTable,
      DoubleSupplier vxOperatorMps,
      DoubleSupplier vyOperatorMps) {
    super(drivetrain, vision, vxOperatorMps, vyOperatorMps);
    m_shooter = shooter;
    m_hopper = hopper;
    m_frontTable = frontTable;
    m_backTable = backTable;
    addRequirements(shooter, hopper);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_settle.reset();
    m_secondsFed = 0.0;
    m_lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    super.execute();
    double now = Timer.getFPGATimestamp();
    double dt = now - m_lastTimestamp;
    m_lastTimestamp = now;

    if (m_solution == null) {
      m_hopper.setRollers(RollerState.OFF);
      DogLog.log("Shooter/ReadyToFire", false);
      DogLog.log("Shooter/BlockedBy", "NO_TARGET");
      return;
    }

    ShotTable table = m_direction == ShotDirection.FRONT ? m_frontTable : m_backTable;
    double distanceFt = Units.metersToFeet(m_solution.shotDistanceMeters());
    ShotTable.Shot shot = table.lookup(distanceFt);
    ShotSetpoint sp = m_direction.setpoint(shot.hoodDeg(), shot.flywheelFps());
    m_shooter.setHoodAngle(Degrees.of(sp.hoodDeg()));
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(sp.flywheelFps()));

    boolean aimed = Math.abs(m_solution.headingError().getDegrees()) <= AimConstants.kAimToleranceDeg;
    boolean stopped = Math.abs(m_drivetrain.getYawRateDegPerSec()) < AimConstants.kSettledRateDegPerSec
        && m_drivetrain.getTranslationalSpeedMps() < AimConstants.kMaxShootSpeedMps;
    boolean settled = m_settle.update(aimed, stopped);
    ShotGate.Status status = ShotGate.evaluate(
        aimed, stopped, settled, m_shooter.isAtTarget() && ShotGate.flywheelFastEnough(sp.flywheelFps()),
        m_vision.getSecondsSinceAccepted(), AimConstants.kMaxVisionAgeToFeedSec);

    boolean feed = status == ShotGate.Status.READY;
    m_hopper.setRollers(feed ? RollerState.FORWARD : RollerState.OFF);
    if (feed) {
      m_secondsFed += dt;
    }

    DogLog.log("Shooter/ReadyToFire", feed);
    DogLog.log("Shooter/BlockedBy", status.name());
    DogLog.log("Shooter/OutOfRange", !table.inRange(distanceFt));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
    m_shooter.idleHood();
    m_hopper.setRollers(RollerState.OFF);
    DogLog.log("Shooter/ReadyToFire", false);
  }

  /** Seconds the hopper has fed during this run (used by the auto named command). */
  public double getSecondsFed() {
    return m_secondsFed;
  }
}
