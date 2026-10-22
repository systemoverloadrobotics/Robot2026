package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper.RollerState;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.junit.jupiter.api.Assertions.assertTrue;

class HopperSimTest {
  @Test
  void testHopperForward() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Hopper hopper = new Hopper();

    hopper.setRollers(RollerState.FORWARD);
    for (int i = 0; i < 100; i++) {
        Unmanaged.feedEnable(100);
        hopper.simulationPeriodic();
        hopper.periodic();
        Timer.delay(0.02);
    }

    double rollerVel = hopper.getRollerMotor().getVelocity().getValueAsDouble();
    double bottomRollerVel = hopper.getBottomRollerMotor().getVelocity().getValueAsDouble();
    double spindexerVel = hopper.getSpindexerMotor().getVelocity().getValueAsDouble();

    // In FORWARD, roller and bottomRoller should spin positively, spindexer negatively
    assertTrue(rollerVel > 0, "Roller velocity should be positive, was " + rollerVel);
    assertTrue(bottomRollerVel > 0, "Bottom roller velocity should be positive, was " + bottomRollerVel);
    assertTrue(spindexerVel < 0, "Spindexer velocity should be negative, was " + spindexerVel);
  }
}
