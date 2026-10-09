package frc.robot.targeting;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Decides whether a vision pose estimate should be fused, and how much to
 * trust it. Pure: no hardware, no PhotonLib types, fully unit-testable.
 *
 * Trust is expressed as standard deviations rather than a yes/no switch.
 * That is deliberate - the XRP prototype hard-switched between vision and
 * odometry, and the discontinuity at every switch was a real bug. Letting
 * the Kalman filter blend continuously removes the switch entirely.
 */
public final class VisionFilter {

  private VisionFilter() {}

  /** @param stdDevs [x, y, theta], meters and radians. Larger = trusted less. */
  public record Decision(boolean accept, String reason, Matrix<N3, N1> stdDevs, boolean rejectedForJump) {
    Decision(boolean accept, String reason, Matrix<N3, N1> stdDevs) {
      this(accept, reason, stdDevs, false);
    }
  }

  private static final Matrix<N3, N1> kRejected = VecBuilder.fill(0, 0, 0);

  public static Decision evaluate(
      Pose3d estimatedPose,
      int tagCount,
      double worstAmbiguity,
      double avgTagDistanceMeters,
      Pose2d currentEstimate,
      boolean hasEverAccepted,
      boolean enforceFieldBounds) {

    if (tagCount <= 0) {
      return new Decision(false, "no tags in estimate", kRejected);
    }

    // Ambiguity only means something for a single-tag solve. A multi-tag PnP
    // solve resolves the two-solution problem geometrically, so its reported
    // per-target ambiguity is not a reason to reject.
    if (tagCount == 1 && worstAmbiguity > VisionConstants.kMaxAmbiguity) {
      return new Decision(false,
          String.format("single-tag ambiguity %.3f > %.3f",
              worstAmbiguity, VisionConstants.kMaxAmbiguity),
          kRejected);
    }

    // The robot is on the carpet. A solve that puts it well above or below
    // the floor is a bad solve - or a wrong robotToCamera height or pitch -
    // and its x/y are no more trustworthy than its z.
    if (Math.abs(estimatedPose.getZ()) > VisionConstants.kMaxPoseZMeters) {
      return new Decision(false,
          String.format("z %.2fm off floor (limit %.2fm)",
              estimatedPose.getZ(), VisionConstants.kMaxPoseZMeters),
          kRejected);
    }

    if (tagCount == 1 && avgTagDistanceMeters > VisionConstants.kMaxSingleTagDistanceMeters) {
      return new Decision(false,
          String.format("single tag %.2fm away > %.2fm",
              avgTagDistanceMeters, VisionConstants.kMaxSingleTagDistanceMeters),
          kRejected);
    }

    // Bounds are only meaningful in the OFFICIAL field frame, whose origin is a
    // field corner. The practice layout's origin is wherever the robot happened
    // to be parked for the mapping session - tags and robot poses behind or to
    // the left of that spot legitimately have negative coordinates, and would
    // all be wrongly rejected here as "outside field bounds".
    if (enforceFieldBounds) {
      double x = estimatedPose.getX();
      double y = estimatedPose.getY();
      if (x < 0.0 || x > FieldConstants.kFieldLengthMeters
          || y < 0.0 || y > FieldConstants.kFieldWidthMeters) {
        return new Decision(false,
            String.format("pose (%.2f, %.2f) outside field bounds", x, y), kRejected);
      }
    }

    // Phoenix odometry boots at (0, 0, 0 deg) and nothing seeds it from vision.
    // If the robot powers on more than kMaxPoseJumpMeters from the origin, the
    // very first estimate would be rejected as an implausible jump - and with
    // the pose never moving toward it, every estimate after that would be
    // rejected too, forever. The jump gate only makes sense once we have a
    // real fused estimate to compare against, so the first accepted estimate
    // bootstraps the pose and skips this check.
    if (hasEverAccepted) {
      double jump = estimatedPose.toPose2d().getTranslation()
          .getDistance(currentEstimate.getTranslation());
      if (jump > VisionConstants.kMaxPoseJumpMeters) {
        return new Decision(false,
            String.format("jump of %.2fm exceeds %.2fm",
                jump, VisionConstants.kMaxPoseJumpMeters),
            kRejected, true);
      }
    }

    // Error in a tag pose solve grows roughly with the square of range, and
    // shrinks when several tags constrain the solution at once.
    double base = tagCount > 1
        ? VisionConstants.kMultiTagBaseStdDevMeters
        : VisionConstants.kSingleTagBaseStdDevMeters;
    double d = Math.max(avgTagDistanceMeters, 0.1);
    double xyStdDev = base * d * d;

    // Heading: 2+ tags pin it geometrically, so trust it (less with distance).
    // A single tag's yaw is the noisiest part of its solve - leave heading to
    // the gyro. See the constants for why the single-tag value is not scaled.
    double thetaStdDev = tagCount > 1
        ? VisionConstants.kMultiTagThetaStdDevRadians * d * d
        : VisionConstants.kSingleTagThetaStdDevRadians;

    return new Decision(true, "accepted", VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
  }
}
