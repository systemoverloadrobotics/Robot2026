package frc.robot.targeting;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.FieldConstants;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Chooses which AprilTag layout the robot believes in.
 *
 * The practice layout describes tags placed by hand around the shop, at
 * positions that do NOT match the official field. Flip
 * FieldConstants.kUsePracticeLayout to swap between them - no other code
 * changes, because everything downstream only ever calls getTagPose(id).
 */
public final class TagFieldLayoutLoader {

  private TagFieldLayoutLoader() {}

  /**
   * The layout requested, plus whether it is actually the one asked for.
   *
   * usedFallback() is what lets a caller fail closed: if the practice layout
   * was requested but unavailable, load() still hands back a usable
   * AprilTagFieldLayout (the official one) so nothing NPEs, but that layout's
   * tag positions are WRONG for the practice space - targeting must not treat
   * it as trustworthy just because it is non-null.
   */
  public record LoadResult(AprilTagFieldLayout layout, boolean usedFallback) {}

  /** The official 2026 REBUILT layout: welded by default, AndyMark when FieldConstants says so. */
  public static AprilTagFieldLayout officialLayout() {
    return officialLayout(FieldConstants.kUseAndyMarkField);
  }

  public static AprilTagFieldLayout officialLayout(boolean andyMark) {
    return AprilTagFieldLayout.loadField(
        andyMark ? AprilTagFields.k2026RebuiltAndymark : AprilTagFields.k2026RebuiltWelded);
  }

  /**
   * The layout the robot should use. Falls back to the official layout, with a
   * loud Driver Station error, if the practice file is selected but missing or
   * unreadable - a silently-empty layout would make every targeting command
   * quietly do nothing. The returned LoadResult.usedFallback() flag is how
   * callers distinguish "using the practice layout as intended" from "using
   * the official layout's real, several-meters-away positions because the
   * practice file could not be read" - the two must not be treated the same.
   */
  public static LoadResult load() {
    if (!FieldConstants.kUsePracticeLayout) {
      return new LoadResult(officialLayout(), false);
    }

    Path path = Filesystem.getDeployDirectory().toPath()
        .resolve(FieldConstants.kPracticeLayoutFile);
    try {
      return new LoadResult(new AprilTagFieldLayout(path), false);
    } catch (IOException e) {
      DriverStation.reportError(
          "Could not load practice layout " + path + " - falling back to the official "
              + "2026 layout. Tag positions will be WRONG for the practice space.",
          e.getStackTrace());
      return new LoadResult(officialLayout(), true);
    }
  }
}
