package frc.robot.util;

/**
 * Driver stick response curve (ported from pinnacles 07631ed).
 *
 * Squaring the stick gives finer control at low speed while full deflection still
 * reaches full speed. The sign is kept so direction and inversions are unchanged.
 */
public final class StickShaping {

  private StickShaping() {}

  /** Returns value squared, with the sign of value. */
  public static double square(double value) {
    return Math.copySign(value * value, value);
  }
}
