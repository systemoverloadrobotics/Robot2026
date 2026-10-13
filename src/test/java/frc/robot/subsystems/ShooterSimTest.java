package frc.robot.subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;

/** The simulator's flywheel model follows the commanded speed and sign (runs ~4 s, real time). */
class ShooterSimTest {
  @Test
  void simulatedFlywheelFollowsCommandedDirection() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    ShooterSubSystem shooter = new ShooterSubSystem();
    for (double target : new double[] {10.0, -10.0}) {
      shooter.setFlywheelVelocity(FeetPerSecond.of(target));
      for (int i = 0; i < 100; i++) {
        Unmanaged.feedEnable(100);
        shooter.simulationPeriodic();
        Timer.delay(0.02);
      }
      assertEquals(target, shooter.getFlywheelVelocity().in(FeetPerSecond), 1.0);
    }
  }
}
