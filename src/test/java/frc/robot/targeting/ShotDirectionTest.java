package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ShotDirectionTest {
  @Test
  void frontFacesTheFiringDirection() {
    assertEquals(30.0, ShotDirection.FRONT.robotHeadingFor(Rotation2d.fromDegrees(30)).getDegrees(), 1e-9);
  }

  @Test
  void backFacesAwayFromTheFiringDirection() {
    assertEquals(-150.0, ShotDirection.BACK.robotHeadingFor(Rotation2d.fromDegrees(30)).getDegrees(), 1e-9);
  }

  @Test
  void setpointSignsFollowTheSpec() {
    assertEquals(new ShotSetpoint(-88.0, 42.0), ShotDirection.FRONT.setpoint(88.0, 42.0));
    assertEquals(new ShotSetpoint(87.0, -42.0), ShotDirection.BACK.setpoint(87.0, 42.0));
  }

  @Test
  void lateralOffsetFlipsForTheBackShot() {
    Translation2d rightSide = new Translation2d(0.0, -0.2);
    assertEquals(-0.2, ShotDirection.FRONT.lateralOffset(rightSide), 1e-9);
    assertEquals(0.2, ShotDirection.BACK.lateralOffset(rightSide), 1e-9);
  }

  @Test
  void otherSwaps() {
    assertEquals(ShotDirection.BACK, ShotDirection.FRONT.other());
    assertEquals(ShotDirection.FRONT, ShotDirection.BACK.other());
  }
}
