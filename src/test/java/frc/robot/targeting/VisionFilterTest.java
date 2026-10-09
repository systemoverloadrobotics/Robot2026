package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class VisionFilterTest {

  private static Pose3d poseAt(double x, double y) {
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  /** The filter only reads the translation, but a null rotation would NPE. */
  private static Pose2d currentAt(double x, double y) {
    return new Pose2d(x, y, Rotation2d.kZero);
  }

  @Test
  void acceptsGoodSingleTagEstimate() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.05, 2.0, currentAt(4.0, 4.0), true, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void rejectsHighAmbiguitySingleTag() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.9, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void ignoresAmbiguityWhenMultipleTagsUsed() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 2, 0.9, 2.0, currentAt(4.0, 4.0), true, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void rejectsPoseOutsideFieldBounds() {
    var d = VisionFilter.evaluate(poseAt(-3.0, 4.0), 2, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void rejectsImplausibleJumpFromCurrentEstimate() {
    var d = VisionFilter.evaluate(poseAt(10.0, 4.0), 1, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void rejectsEstimateWithNoTags() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 0, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void farEstimateIsTrustedLessThanCloseOne() {
    var close = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 1.0, currentAt(4.0, 4.0), true, true);
    var far = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 3.0, currentAt(4.0, 4.0), true, true);
    assertTrue(far.stdDevs().get(0, 0) > close.stdDevs().get(0, 0));
  }

  @Test
  void multiTagIsTrustedMoreThanSingleTagAtSameDistance() {
    var single = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    var multi = VisionFilter.evaluate(poseAt(4.0, 4.0), 3, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertTrue(multi.stdDevs().get(0, 0) < single.stdDevs().get(0, 0));
  }

  @Test
  void farAwayFirstEstimateIsAcceptedWhenNothingHasEverBeenAccepted() {
    // Phoenix odometry boots at (0, 0, 0deg). If the robot powers on far from
    // the origin, the jump gate must not block the very first estimate - that
    // would leave the fused pose stuck at the origin forever, since nothing
    // else can move it toward the truth.
    var d = VisionFilter.evaluate(
        poseAt(10.0, 4.0), 1, 0.0, 2.0, currentAt(0.0, 0.0), false, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void sameFarAwayEstimateIsRejectedOnceSomethingHasBeenAccepted() {
    var d = VisionFilter.evaluate(
        poseAt(10.0, 4.0), 1, 0.0, 2.0, currentAt(0.0, 0.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void negativeCoordinatePoseIsAcceptedWhenFieldBoundsNotEnforced() {
    // The practice layout's origin is wherever the robot was parked for the
    // mapping session, not a field corner - legitimately negative coordinates
    // must not be rejected there.
    var d = VisionFilter.evaluate(
        poseAt(-3.0, -2.0), 1, 0.0, 2.0, currentAt(-3.0, -2.0), false, false);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void negativeCoordinatePoseIsStillRejectedWhenFieldBoundsEnforced() {
    var d = VisionFilter.evaluate(
        poseAt(-3.0, -2.0), 1, 0.0, 2.0, currentAt(-3.0, -2.0), false, true);
    assertFalse(d.accept());
  }

  @Test
  void rejectsPoseFarOffTheFloor() {
    var d = VisionFilter.evaluate(
        new Pose3d(4.0, 4.0, 0.5, new Rotation3d()), 2, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void acceptsPoseSlightlyOffTheFloor() {
    var d = VisionFilter.evaluate(
        new Pose3d(4.0, 4.0, 0.05, new Rotation3d()), 2, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void rejectsSingleTagBeyondMaxDistance() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 6.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void multiTagHasNoDistanceLimit() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 2, 0.0, 6.0, currentAt(4.0, 4.0), true, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void multiTagHeadingIsTrustedFarMoreThanSingleTag() {
    var single = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 1.0, currentAt(4.0, 4.0), true, true);
    var multi = VisionFilter.evaluate(poseAt(4.0, 4.0), 2, 0.0, 1.0, currentAt(4.0, 4.0), true, true);
    assertTrue(multi.stdDevs().get(2, 0) * 10 < single.stdDevs().get(2, 0));
  }

  @Test
  void closeSingleTagStillDoesNotSteerHeading() {
    // Distance-scaling the single-tag heading std dev would make a tag 0.3 m
    // away trusted for heading - the noisiest part of a one-tag solve.
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 0.3, currentAt(4.0, 4.0), true, true);
    assertTrue(d.stdDevs().get(2, 0) >= 1.0);
  }

  @Test
  void jumpRejectionIsFlagged() {
    var jump = VisionFilter.evaluate(poseAt(10.0, 4.0), 2, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(jump.accept());
    assertTrue(jump.rejectedForJump());
    var ambiguous = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.9, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(ambiguous.rejectedForJump());
  }
}
