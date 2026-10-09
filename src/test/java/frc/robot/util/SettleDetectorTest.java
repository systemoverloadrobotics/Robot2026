package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SettleDetectorTest {

  @Test
  void requiresConsecutiveQualifyingLoops() {
    var d = new SettleDetector(3);
    assertFalse(d.update(true, true));
    assertFalse(d.update(true, true));
    assertTrue(d.update(true, true));
  }

  @Test
  void withinToleranceButStillMovingDoesNotCount() {
    var d = new SettleDetector(2);
    assertFalse(d.update(true, false));
    assertFalse(d.update(true, false));
    assertEquals(0, d.consecutiveLoops());
  }

  @Test
  void stoppedButOutOfToleranceDoesNotCount() {
    var d = new SettleDetector(2);
    assertFalse(d.update(false, true));
    assertEquals(0, d.consecutiveLoops());
  }

  @Test
  void oneBadLoopResetsTheStreak() {
    var d = new SettleDetector(3);
    d.update(true, true);
    d.update(true, true);
    assertFalse(d.update(false, true));
    assertEquals(0, d.consecutiveLoops());
    assertFalse(d.update(true, true));
  }

  @Test
  void resetClearsTheStreak() {
    var d = new SettleDetector(2);
    d.update(true, true);
    d.reset();
    assertFalse(d.update(true, true));
  }
}
