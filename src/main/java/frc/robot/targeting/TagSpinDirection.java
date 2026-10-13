package frc.robot.targeting;

import java.util.Optional;

/**
 * Home-test rule: which way to spin the flywheel from which camera sees a tag.
 *
 * Front camera only -> FRONT (+, CCW); back camera only -> BACK (-, CW). Neither or both -> stop:
 * with both cameras seeing tags the direction is ambiguous, so the flywheel doesn't guess.
 */
public final class TagSpinDirection {
  private TagSpinDirection() {}

  /** The chosen direction (empty = stop) and the label logged for it. */
  public record Choice(Optional<ShotDirection> direction, String label) {
    /** Signed flywheel speed for a test speed magnitude; 0 when stopped. */
    public double flywheelFps(double testFpsMagnitude) {
      return direction.map(d -> d.setpoint(0.0, testFpsMagnitude).flywheelFps()).orElse(0.0);
    }
  }

  public static Choice choose(boolean frontSeesTag, boolean backSeesTag) {
    if (frontSeesTag && backSeesTag) {
      return new Choice(Optional.empty(), "BOTH_CAMERAS");
    }
    if (frontSeesTag) {
      return new Choice(Optional.of(ShotDirection.FRONT), "FRONT");
    }
    if (backSeesTag) {
      return new Choice(Optional.of(ShotDirection.BACK), "BACK");
    }
    return new Choice(Optional.empty(), "NO_TAG");
  }
}
