package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.dashboard.ShotStatus;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.targeting.ShotGate;
import frc.robot.targeting.ShotSetpoint;
import java.util.function.Supplier;

/**
 * Hold-to-run shot without aiming: spin to the supplied setpoint, feed whenever the shooter is
 * on target, stop everything on release. Used by MANUAL (shot table at the driver's distance)
 * and CALIBRATION (hand-set values).
 */
public class ShootCommand extends Command {
  private final ShooterSubSystem m_shooter;
  private final Hopper m_hopper;
  private final Supplier<ShotSetpoint> m_setpoint;

  public ShootCommand(ShooterSubSystem shooter, Hopper hopper, Supplier<ShotSetpoint> setpoint) {
    m_shooter = shooter;
    m_hopper = hopper;
    m_setpoint = setpoint;
    addRequirements(shooter, hopper);
  }

  @Override
  public void execute() {
    ShotSetpoint sp = m_setpoint.get();
    m_shooter.setHoodAngle(Degrees.of(sp.hoodDeg()));
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(sp.flywheelFps()));
    boolean ready = m_shooter.isAtTarget() && ShotGate.flywheelFastEnough(sp.flywheelFps());
    m_hopper.setRollers(ready ? RollerState.FORWARD : RollerState.OFF);
    DogLog.log("Shooter/ReadyToFire", ready);
    ShotStatus.setReady(ready, ready ? "READY" : "SHOOTER_NOT_READY");
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
    m_shooter.idleHood();
    m_hopper.setRollers(RollerState.OFF);
    DogLog.log("Shooter/ReadyToFire", false);
    ShotStatus.clear();
  }
}
