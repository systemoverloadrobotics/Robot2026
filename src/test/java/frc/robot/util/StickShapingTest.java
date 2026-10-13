package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class StickShapingTest {
  @Test
  void squaresMagnitudeAndKeepsSign() {
    assertEquals(0.25, StickShaping.square(0.5), 1e-9);
    assertEquals(-0.25, StickShaping.square(-0.5), 1e-9);
  }

  @Test
  void keepsZeroAndFullDeflection() {
    assertEquals(0.0, StickShaping.square(0.0), 1e-9);
    assertEquals(1.0, StickShaping.square(1.0), 1e-9);
    assertEquals(-1.0, StickShaping.square(-1.0), 1e-9);
  }
}
