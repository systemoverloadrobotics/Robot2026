package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.targeting.ShotGate.Status;
import org.junit.jupiter.api.Test;

class ShotGateTest {
  private static Status eval(boolean aimed, boolean stopped, boolean settled, boolean atTarget, double visionAge) {
    return ShotGate.evaluate(aimed, stopped, settled, atTarget, visionAge, 1.0);
  }

  @Test
  void allConditionsMetIsReady() {
    assertEquals(Status.READY, eval(true, true, true, true, 0.2));
  }

  @Test
  void eachConditionBlocksOnItsOwn() {
    assertEquals(Status.NOT_AIMED, eval(false, true, true, true, 0.2));
    assertEquals(Status.MOVING, eval(true, false, true, true, 0.2));
    assertEquals(Status.SETTLING, eval(true, true, false, true, 0.2));
    assertEquals(Status.SHOOTER_NOT_READY, eval(true, true, true, false, 0.2));
    assertEquals(Status.VISION_STALE, eval(true, true, true, true, 1.5));
  }

  @Test
  void neverSeenVisionIsStale() {
    assertEquals(Status.VISION_STALE, eval(true, true, true, true, Double.POSITIVE_INFINITY));
  }

  @Test
  void slowFlywheelIsNotFastEnoughToFeed() {
    assertFalse(ShotGate.flywheelFastEnough(0.0));
    assertFalse(ShotGate.flywheelFastEnough(-4.9));
    assertTrue(ShotGate.flywheelFastEnough(-5.0));
    assertTrue(ShotGate.flywheelFastEnough(42.0));
  }
}
