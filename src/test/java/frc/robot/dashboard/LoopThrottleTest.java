package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class LoopThrottleTest {

  @Test
  void firesOnFirstLoopThenEveryNth() {
    var t = new LoopThrottle(3);
    assertTrue(t.tick());
    assertFalse(t.tick());
    assertFalse(t.tick());
    assertTrue(t.tick());
  }

  @Test
  void everyOneFiresEveryLoop() {
    var t = new LoopThrottle(1);
    assertTrue(t.tick());
    assertTrue(t.tick());
  }
}
