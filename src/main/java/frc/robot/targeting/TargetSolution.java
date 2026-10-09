package frc.robot.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Turns "where I am" plus "where the target is" into the two numbers any
 * consumer needs: how far, and which way.
 *
 * THIS IS THE SHOOTER SEAM. Today DriveToTagCommand consumes distance and
 * bearing to drive to a standoff. When the shooter is mounted, a ShootCommand
 * consumes the SAME two numbers - distance feeds a lookup table for flywheel
 * RPM and hood angle, bearing aims the chassis. Nothing in this class changes.
 *
 * Pure by design: no hardware, no subsystems, no WPILib HAL. Every bit of
 * geometry that had a real bug on the XRP prototype lives here where it can be
 * unit-tested off-robot.
 */
public record TargetSolution(double distanceMeters, Rotation2d bearing,
                             Translation2d toTarget) {

  /**
   * @param robotPose the FUSED pose (odometry + vision), not a raw camera reading
   * @param target    the target's field position, from the AprilTag layout
   */
  public static TargetSolution solve(Pose2d robotPose, Translation2d target) {
    Translation2d toTarget = target.minus(robotPose.getTranslation());
    return new TargetSolution(toTarget.getNorm(), toTarget.getAngle(), toTarget);
  }

  /**
   * How far the robot still has to rotate to face the target.
   *
   * Uses Rotation2d.minus(), never plain subtraction - raw angle subtraction
   * breaks across the +-180 wraparound, which is exactly how a single 10 degree
   * turn once reported as -172 degrees on the prototype.
   */
  public Rotation2d headingError(Rotation2d currentHeading) {
    return bearing.minus(currentHeading);
  }

  /** Positive when the robot is farther from the target than the standoff. */
  public double rangeErrorMeters(double standoffMeters) {
    return distanceMeters - standoffMeters;
  }

  /** Field-relative unit vector pointing at the target; zero if already there. */
  public Translation2d unitVectorToTarget() {
    if (distanceMeters < 1e-9) {
      return Translation2d.kZero;
    }
    return toTarget.div(distanceMeters);
  }
}
