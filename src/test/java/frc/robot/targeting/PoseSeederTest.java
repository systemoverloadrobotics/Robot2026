package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.targeting.PoseSeeder.Action;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class PoseSeederTest {
  private static Pose2d at(double x, double y) {
    return new Pose2d(x, y, Rotation2d.kZero);
  }

  @Test
  void firstMultiTagSnapsTheWholePose() {
    var s = new PoseSeeder(3, 0.3);
    assertFalse(s.isTranslationSeeded());
    assertEquals(Action.RESET_POSE, s.onAccepted(2));
    assertTrue(s.isTranslationSeeded());
    assertEquals(Action.BLEND, s.onAccepted(2));
    assertEquals(Action.BLEND, s.onAccepted(1));
  }

  @Test
  void singleTagSnapsTranslationThenFirstMultiTagSnapsHeading() {
    var s = new PoseSeeder(3, 0.3);
    assertEquals(Action.RESET_TRANSLATION, s.onAccepted(1));
    assertEquals(Action.BLEND, s.onAccepted(1));
    assertEquals(Action.RESET_POSE, s.onAccepted(3));
    assertEquals(Action.BLEND, s.onAccepted(3));
  }

  @Test
  void consistentJumpRejectionsTriggerAReset() {
    var s = new PoseSeeder(3, 0.3);
    s.onAccepted(2);
    assertEquals(Optional.empty(), s.onJumpRejected(at(5.0, 4.0), 2));
    assertEquals(Optional.empty(), s.onJumpRejected(at(5.1, 4.0), 2));
    assertEquals(Optional.of(Action.RESET_POSE), s.onJumpRejected(at(5.0, 4.1), 2));
    // the count starts over after a reset
    assertEquals(Optional.empty(), s.onJumpRejected(at(5.0, 4.0), 2));
  }

  @Test
  void singleTagRecoveryKeepsTheGyroHeading() {
    var s = new PoseSeeder(2, 0.3);
    s.onAccepted(2);
    s.onJumpRejected(at(5.0, 4.0), 1);
    assertEquals(Optional.of(Action.RESET_TRANSLATION), s.onJumpRejected(at(5.0, 4.0), 1));
  }

  @Test
  void inconsistentRejectionsRestartTheCount() {
    var s = new PoseSeeder(2, 0.3);
    s.onAccepted(2);
    s.onJumpRejected(at(5.0, 4.0), 2);
    assertEquals(Optional.empty(), s.onJumpRejected(at(8.0, 1.0), 2));
    assertEquals(Optional.of(Action.RESET_POSE), s.onJumpRejected(at(8.0, 1.1), 2));
  }

  @Test
  void anAcceptedEstimateClearsPendingRejections() {
    var s = new PoseSeeder(2, 0.3);
    s.onAccepted(2);
    s.onJumpRejected(at(5.0, 4.0), 2);
    s.onAccepted(2);
    assertEquals(Optional.empty(), s.onJumpRejected(at(5.0, 4.0), 2));
  }
}
