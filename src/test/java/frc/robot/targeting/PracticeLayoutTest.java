package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.io.IOException;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/** deploy/practice-layout.json: the lab hub, centered on (0, 0), for lab mode. */
class PracticeLayoutTest {
  private static AprilTagFieldLayout load() throws IOException {
    return new AprilTagFieldLayout(
        Path.of("src/main/deploy").resolve(FieldConstants.kPracticeLayoutFile));
  }

  @Test
  void labTargetIsTheHubCenter() throws IOException {
    Pose3d target = load().getTagPose(FieldConstants.kLabTargetTagId).orElseThrow();
    assertEquals(0.0, target.getX(), 1e-3);
    assertEquals(0.0, target.getY(), 1e-3);
  }

  @Test
  void bothHubTagSetsAreCenteredOnTheOrigin() throws IOException {
    for (Alliance alliance : Alliance.values()) {
      Translation2d c = FieldTargets.hubCenter(load(), alliance).orElseThrow();
      assertEquals(0.0, c.getX(), 1e-3, alliance.name());
      assertEquals(0.0, c.getY(), 1e-3, alliance.name());
    }
  }

  @Test
  void hubTagsKeepTheirOfficialSpacingAndHeight() throws IOException {
    AprilTagFieldLayout lab = load();
    AprilTagFieldLayout official = TagFieldLayoutLoader.officialLayout(false);
    Pose3d lab19 = lab.getTagPose(19).orElseThrow();
    Pose3d lab20 = lab.getTagPose(20).orElseThrow();
    Pose3d off19 = official.getTagPose(19).orElseThrow();
    Pose3d off20 = official.getTagPose(20).orElseThrow();
    assertEquals(
        off19.getTranslation().getDistance(off20.getTranslation()),
        lab19.getTranslation().getDistance(lab20.getTranslation()),
        1e-3);
    assertEquals(off19.getZ(), lab19.getZ(), 1e-3);
    assertEquals(off19.getRotation().getZ(), lab19.getRotation().getZ(), 1e-6);
    assertTrue(lab.getTags().size() == 17, "8 blue + 8 red hub tags + the center target");
  }
}
