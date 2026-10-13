package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TagSpinDirectionTest {
  @Test
  void frontCameraOnlySpinsFront() {
    TagSpinDirection.Choice c = TagSpinDirection.choose(true, false);
    assertEquals("FRONT", c.label());
    assertEquals(10.0, c.flywheelFps(10.0), 1e-9);
  }

  @Test
  void backCameraOnlySpinsBack() {
    TagSpinDirection.Choice c = TagSpinDirection.choose(false, true);
    assertEquals("BACK", c.label());
    assertEquals(-10.0, c.flywheelFps(10.0), 1e-9);
  }

  @Test
  void noTagStops() {
    TagSpinDirection.Choice c = TagSpinDirection.choose(false, false);
    assertEquals("NO_TAG", c.label());
    assertEquals(0.0, c.flywheelFps(10.0), 1e-9);
  }

  @Test
  void bothCamerasStopBecauseTheDirectionIsAmbiguous() {
    TagSpinDirection.Choice c = TagSpinDirection.choose(true, true);
    assertEquals("BOTH_CAMERAS", c.label());
    assertEquals(0.0, c.flywheelFps(10.0), 1e-9);
  }
}
