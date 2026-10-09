package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.targeting.ShotDirection;
import frc.robot.targeting.ShotSetpoint;
import frc.robot.targeting.ShotTable;
import org.junit.jupiter.api.Test;

class CalibrationStateTest {
  private static CalibrationState state() {
    ShotTable front = new ShotTable(10.0, 80.0).put(6.0, 40.0, 30.0).put(12.0, 60.0, 50.0);
    ShotTable back = new ShotTable(20.0, 85.0).put(6.0, 45.0, 35.0).put(12.0, 65.0, 55.0);
    return new CalibrationState(front, back);
  }

  @Test
  void startsFrontAtTheTableEntry() {
    var s = state();
    s.resetToTable(9.0);
    assertEquals(ShotDirection.FRONT, s.direction());
    assertEquals(50.0, s.hoodDeg(), 1e-9);
    assertEquals(40.0, s.flywheelFps(), 1e-9);
  }

  @Test
  void toggleSwitchesDirectionAndResetsToThatTable() {
    var s = state();
    s.toggleDirection(9.0);
    assertEquals(ShotDirection.BACK, s.direction());
    assertEquals(55.0, s.hoodDeg(), 1e-9);
    assertEquals(new ShotSetpoint(55.0, -45.0), s.setpoint());
  }

  @Test
  void stepsChangeMagnitudes() {
    var s = state();
    s.resetToTable(6.0);
    s.stepHood(2.5);
    s.stepFlywheel(-2.0);
    assertEquals(new ShotSetpoint(-42.5, 28.0), s.setpoint());
  }

  @Test
  void typedValuesAreClamped() {
    var s = state();
    s.setHoodDeg(120.0);
    assertEquals(80.0, s.hoodDeg(), 1e-9);
    s.setHoodDeg(-5.0);
    assertEquals(10.0, s.hoodDeg(), 1e-9);
    s.setFlywheelFps(-10.0);
    assertEquals(0.0, s.flywheelFps(), 1e-9);
    s.setFlywheelFps(500.0);
    assertEquals(90.0, s.flywheelFps(), 1e-9);
    s.setManualDistanceFt(-3.0);
    assertEquals(0.0, s.manualDistanceFt(), 1e-9);
  }

  @Test
  void typedNegativeValuesUseTheirMagnitude() {
    var s = state();
    s.setTypedHoodDeg(-55.0);
    assertEquals(55.0, s.hoodDeg(), 1e-9);
    s.setTypedFlywheelFps(-42.0);
    assertEquals(42.0, s.flywheelFps(), 1e-9);
  }

  @Test
  void stepsBelowTheMinimumStillClampInsteadOfFlippingSign() {
    var s = state();
    s.setHoodDeg(11.0);
    s.stepHood(-2.5);
    assertEquals(10.0, s.hoodDeg(), 1e-9);
  }
}
