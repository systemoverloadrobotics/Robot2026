package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class TargetSolutionTest {

  private static final double EPS = 1e-6;

  @Test
  void distanceIsEuclidean() {
    var s = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.kZero), new Translation2d(3, 4));
    assertEquals(5.0, s.distanceMeters(), EPS);
  }

  @Test
  void bearingIsIndependentOfRobotHeading() {
    var facingZero = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.kZero), new Translation2d(1, 0));
    var facingNinety = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)), new Translation2d(1, 0));
    assertEquals(0.0, facingZero.bearing().getDegrees(), EPS);
    assertEquals(0.0, facingNinety.bearing().getDegrees(), EPS);
  }

  @Test
  void bearingPointsAtTargetInAllQuadrants() {
    var origin = new Pose2d(0, 0, Rotation2d.kZero);
    assertEquals(45.0,
        TargetSolution.solve(origin, new Translation2d(1, 1)).bearing().getDegrees(), EPS);
    assertEquals(135.0,
        TargetSolution.solve(origin, new Translation2d(-1, 1)).bearing().getDegrees(), EPS);
    assertEquals(-135.0,
        TargetSolution.solve(origin, new Translation2d(-1, -1)).bearing().getDegrees(), EPS);
    assertEquals(-45.0,
        TargetSolution.solve(origin, new Translation2d(1, -1)).bearing().getDegrees(), EPS);
  }

  @Test
  void headingErrorTakesTheShortWayAroundTheWrap() {
    var s = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.kZero), new Translation2d(-1, 0.0001));
    // bearing is just under +180; current heading is just under -180.
    double error = s.headingError(Rotation2d.fromDegrees(-179)).getDegrees();
    assertEquals(-1.0, error, 0.05);
  }

  @Test
  void headingErrorIsZeroWhenAlreadyAimed() {
    var s = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.fromDegrees(30)), new Translation2d(1, 0));
    assertEquals(-30.0, s.headingError(Rotation2d.fromDegrees(30)).getDegrees(), EPS);
  }

  @Test
  void rangeErrorIsPositiveWhenTooFar() {
    var s = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.kZero), new Translation2d(5, 0));
    assertEquals(3.0, s.rangeErrorMeters(2.0), EPS);
  }

  @Test
  void rangeErrorIsNegativeWhenTooClose() {
    var s = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.kZero), new Translation2d(1, 0));
    assertEquals(-1.0, s.rangeErrorMeters(2.0), EPS);
  }

  @Test
  void unitVectorHasLengthOne() {
    var s = TargetSolution.solve(
        new Pose2d(0, 0, Rotation2d.kZero), new Translation2d(3, 4));
    assertEquals(1.0, s.unitVectorToTarget().getNorm(), EPS);
    assertEquals(0.6, s.unitVectorToTarget().getX(), EPS);
    assertEquals(0.8, s.unitVectorToTarget().getY(), EPS);
  }

  @Test
  void unitVectorIsSafeWhenExactlyOnTheTarget() {
    var s = TargetSolution.solve(
        new Pose2d(2, 2, Rotation2d.kZero), new Translation2d(2, 2));
    assertEquals(0.0, s.unitVectorToTarget().getNorm(), EPS);
  }
}
