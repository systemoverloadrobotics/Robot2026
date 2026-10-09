package frc.robot.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Aim geometry for Arc-imedes' bi-directional shooter. Pure: no hardware, unit-tested.
 *
 * The fuel launches from a point on the right side of the robot, not the center, so the robot
 * turns until the line from that launch point along the firing direction passes through the
 * target: firing = bearing - asin(e / D), where e is the launch point's offset to the left of the
 * firing line and D the center-to-target distance.
 */
public final class HubAim {
  private HubAim() {}

  public record Solution(
      ShotDirection direction,
      Rotation2d robotHeading,
      Rotation2d headingError,
      double shotDistanceMeters) {}

  public static Solution solve(
      Pose2d robotPose, Translation2d target, ShotDirection direction, Translation2d launchPoint) {
    TargetSolution toTarget = TargetSolution.solve(robotPose, target);
    double d = toTarget.distanceMeters();
    double ratio = d < 1e-6 ? 0.0 : MathUtil.clamp(direction.lateralOffset(launchPoint) / d, -1.0, 1.0);
    Rotation2d firing = toTarget.bearing().minus(Rotation2d.fromRadians(Math.asin(ratio)));
    Rotation2d heading = direction.robotHeadingFor(firing);
    Rotation2d error = heading.minus(robotPose.getRotation());
    Translation2d launchField = robotPose.getTranslation().plus(launchPoint.rotateBy(heading));
    return new Solution(direction, heading, error, target.getDistance(launchField));
  }

  /** The end needing the smaller turn. Ties go to FRONT. */
  public static ShotDirection chooseDirection(
      Pose2d robotPose, Translation2d target, Translation2d frontLaunch, Translation2d backLaunch) {
    double front = Math.abs(solve(robotPose, target, ShotDirection.FRONT, frontLaunch).headingError().getRadians());
    double back = Math.abs(solve(robotPose, target, ShotDirection.BACK, backLaunch).headingError().getRadians());
    return back < front - 1e-9 ? ShotDirection.BACK : ShotDirection.FRONT;
  }

  /** Launch point to target at the robot's CURRENT heading (used by calibration). */
  public static double currentShotDistanceMeters(
      Pose2d robotPose, Translation2d target, Translation2d launchPoint) {
    Translation2d launchField =
        robotPose.getTranslation().plus(launchPoint.rotateBy(robotPose.getRotation()));
    return target.getDistance(launchField);
  }

  /**
   * Driver stick velocities are operator-relative; the aim request is blue-origin field-relative.
   * On red the driver faces the other way, so both axes flip.
   */
  public static Translation2d operatorToField(double vx, double vy, boolean redAlliance) {
    return redAlliance ? new Translation2d(-vx, -vy) : new Translation2d(vx, vy);
  }
}
