package frc.robot.targeting;

import frc.robot.Constants.ShotConstants;

/** Whether align-and-shoot may feed fuel, and if not, the first reason why not. Pure. */
public final class ShotGate {
  private ShotGate() {}

  public enum Status { READY, NOT_AIMED, MOVING, SETTLING, SHOOTER_NOT_READY, VISION_STALE }

  /**
   * True when the commanded flywheel speed is high enough to shoot. A near-zero target counts as
   * "at target" once the wheel stops, which would feed fuel into a stopped flywheel.
   */
  public static boolean flywheelFastEnough(double targetFlywheelFps) {
    return Math.abs(targetFlywheelFps) >= ShotConstants.kMinFeedFlywheelFps;
  }

  public static Status evaluate(
      boolean aimed,
      boolean stopped,
      boolean settled,
      boolean shooterAtTarget,
      double secondsSinceVision,
      double maxVisionAgeSec) {
    if (!aimed) {
      return Status.NOT_AIMED;
    }
    if (!stopped) {
      return Status.MOVING;
    }
    if (!settled) {
      return Status.SETTLING;
    }
    if (!shooterAtTarget) {
      return Status.SHOOTER_NOT_READY;
    }
    if (!(secondsSinceVision <= maxVisionAgeSec)) {
      return Status.VISION_STALE;
    }
    return Status.READY;
  }
}
