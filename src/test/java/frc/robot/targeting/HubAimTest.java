package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class HubAimTest {
  private static final Translation2d kNone = new Translation2d();
  private static final Translation2d kHub = new Translation2d(10.0, 0.0);

  @Test
  void zeroOffsetFrontAimsStraightAtTarget() {
    var s = HubAim.solve(new Pose2d(0, 0, Rotation2d.fromDegrees(20)), kHub, ShotDirection.FRONT, kNone);
    assertEquals(0.0, s.robotHeading().getDegrees(), 1e-9);
    assertEquals(-20.0, s.headingError().getDegrees(), 1e-9);
    assertEquals(10.0, s.shotDistanceMeters(), 1e-9);
  }

  @Test
  void zeroOffsetBackPointsTheBackAtTarget() {
    var s = HubAim.solve(new Pose2d(0, 0, Rotation2d.kZero), kHub, ShotDirection.BACK, kNone);
    assertEquals(180.0, Math.abs(s.robotHeading().getDegrees()), 1e-9);
  }

  @Test
  void rightSideLaunchPointLinesUpWithTheTarget() {
    for (ShotDirection dir : ShotDirection.values()) {
      Translation2d launch = new Translation2d(0.1, -0.3);
      Pose2d robot = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(50));
      var s = HubAim.solve(robot, kHub, dir, launch);
      Translation2d launchField = robot.getTranslation().plus(launch.rotateBy(s.robotHeading()));
      Rotation2d firing = kHub.minus(launchField).getAngle();
      Rotation2d expectedHeading = dir.robotHeadingFor(firing);
      assertEquals(0.0, expectedHeading.minus(s.robotHeading()).getDegrees(), 1e-6, dir.name());
      assertEquals(kHub.getDistance(launchField), s.shotDistanceMeters(), 1e-9, dir.name());
    }
  }

  @Test
  void headingErrorWrapsAcross180() {
    var s = HubAim.solve(new Pose2d(20, 0, Rotation2d.fromDegrees(170)), kHub, ShotDirection.FRONT, kNone);
    assertEquals(10.0, s.headingError().getDegrees(), 1e-9);
  }

  @Test
  void facingTheHubPicksFront() {
    assertEquals(ShotDirection.FRONT,
        HubAim.chooseDirection(new Pose2d(0, 0, Rotation2d.fromDegrees(30)), kHub, kNone, kNone));
  }

  @Test
  void facingAwayPicksBack() {
    assertEquals(ShotDirection.BACK,
        HubAim.chooseDirection(new Pose2d(0, 0, Rotation2d.fromDegrees(150)), kHub, kNone, kNone));
  }

  @Test
  void exactlySidewaysPicksFront() {
    assertEquals(ShotDirection.FRONT,
        HubAim.chooseDirection(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), kHub, kNone, kNone));
  }

  @Test
  void launchOffsetLargerThanDistanceDoesNotProduceNaN() {
    var s = HubAim.solve(new Pose2d(9.9, 0, Rotation2d.kZero), kHub, ShotDirection.FRONT, new Translation2d(0, -0.5));
    assertFalse(Double.isNaN(s.robotHeading().getRadians()));
    var atTarget = HubAim.solve(new Pose2d(10, 0, Rotation2d.kZero), kHub, ShotDirection.FRONT, kNone);
    assertFalse(Double.isNaN(atTarget.robotHeading().getRadians()));
  }

  @Test
  void currentShotDistanceUsesTheCurrentHeading() {
    Pose2d robot = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    // launch point 1 m forward -> at heading 90 it sits at (0, 1)
    assertEquals(Math.hypot(10, 1), HubAim.currentShotDistanceMeters(robot, kHub, new Translation2d(1, 0)), 1e-9);
  }

  @Test
  void operatorToFieldFlipsOnRed() {
    assertEquals(new Translation2d(1, 2), HubAim.operatorToField(1, 2, false));
    assertEquals(new Translation2d(-1, -2), HubAim.operatorToField(1, 2, true));
  }
}
