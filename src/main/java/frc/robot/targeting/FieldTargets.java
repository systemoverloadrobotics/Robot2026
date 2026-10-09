package frc.robot.targeting;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

/** Field points the robot aims at, derived from the AprilTag layout rather than hand-typed. */
public final class FieldTargets {
  private FieldTargets() {}

  /** Tags on each alliance's hub in the 2026 REBUILT layout (all at 1.124 m height). */
  public static final int[] kBlueHubTags = {18, 19, 20, 21, 24, 25, 26, 27};
  public static final int[] kRedHubTags = {2, 3, 4, 5, 8, 9, 10, 11};

  /**
   * Center of the alliance's hub: the middle of the bounding box of its tags. Empty if any hub
   * tag is missing from the layout (for example, a practice layout).
   */
  public static Optional<Translation2d> hubCenter(AprilTagFieldLayout layout, Alliance alliance) {
    int[] ids = alliance == Alliance.Red ? kRedHubTags : kBlueHubTags;
    double minX = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (int id : ids) {
      Optional<Pose3d> pose = layout.getTagPose(id);
      if (pose.isEmpty()) {
        return Optional.empty();
      }
      minX = Math.min(minX, pose.get().getX());
      maxX = Math.max(maxX, pose.get().getX());
      minY = Math.min(minY, pose.get().getY());
      maxY = Math.max(maxY, pose.get().getY());
    }
    return Optional.of(new Translation2d((minX + maxX) / 2.0, (minY + maxY) / 2.0));
  }
}
