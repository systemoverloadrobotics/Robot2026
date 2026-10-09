package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.targeting.TagFieldLayoutLoader.LoadResult;
import org.junit.jupiter.api.Test;

class TagFieldLayoutLoaderTest {

  @Test
  void officialLayoutHasThirtyTwoTags() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout();
    assertEquals(32, layout.getTags().size());
  }

  @Test
  void officialLayoutHasCorrectFieldDimensions() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout();
    assertEquals(16.541, layout.getFieldLength(), 0.001);
    assertEquals(8.069, layout.getFieldWidth(), 0.001);
  }

  @Test
  void officialLayoutContainsTagSixteen() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout();
    assertTrue(layout.getTagPose(16).isPresent());
  }

  @Test
  void missingTagIdReturnsEmpty() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout();
    assertTrue(layout.getTagPose(999).isEmpty());
  }

  // load() itself needs Filesystem.getDeployDirectory() and the HAL, so it
  // cannot run as a plain unit test. What IS pure and testable is the
  // LoadResult record load() hands back - specifically, that usedFallback()
  // correctly distinguishes "this is the layout that was asked for" from
  // "this is the official layout standing in because the requested one could
  // not be read", which is exactly the flag VisionSubsystem.getTargetPosition
  // relies on to fail closed.
  @Test
  void loadResultReportsNoFallbackForThePrimaryLayout() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout();
    LoadResult result = new LoadResult(layout, false);
    assertFalse(result.usedFallback());
    assertEquals(layout, result.layout());
  }

  @Test
  void loadResultReportsFallbackWhenFlagged() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout();
    LoadResult result = new LoadResult(layout, true);
    assertTrue(result.usedFallback());
    assertEquals(layout, result.layout());
  }

  @Test
  void andyMarkLayoutIsSlightlySmaller() {
    AprilTagFieldLayout layout = TagFieldLayoutLoader.officialLayout(true);
    assertEquals(16.518, layout.getFieldLength(), 0.001);
  }
}
