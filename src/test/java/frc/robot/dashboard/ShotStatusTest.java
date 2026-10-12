package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotStatusTest {

  @Test
  void clearReturnsToIdle() {
    ShotStatus.setAimed(true);
    ShotStatus.setReady(true, "READY");
    ShotStatus.clear();
    assertFalse(ShotStatus.aimed());
    assertFalse(ShotStatus.ready());
    assertEquals("IDLE", ShotStatus.blockedBy());
  }

  @Test
  void keepsLatestValues() {
    ShotStatus.setAimed(true);
    ShotStatus.setReady(false, "MOVING");
    assertTrue(ShotStatus.aimed());
    assertEquals("MOVING", ShotStatus.blockedBy());
    ShotStatus.clear();
  }
}
