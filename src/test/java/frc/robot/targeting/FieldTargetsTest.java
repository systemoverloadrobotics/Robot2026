package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import org.junit.jupiter.api.Test;

class FieldTargetsTest {
  @Test
  void weldedBlueHubCenter() {
    Translation2d c = FieldTargets.hubCenter(TagFieldLayoutLoader.officialLayout(false), Alliance.Blue).orElseThrow();
    assertEquals(4.6255, c.getX(), 0.002);
    assertEquals(4.0345, c.getY(), 0.002);
  }

  @Test
  void weldedRedHubCenter() {
    Translation2d c = FieldTargets.hubCenter(TagFieldLayoutLoader.officialLayout(false), Alliance.Red).orElseThrow();
    assertEquals(11.9155, c.getX(), 0.002);
    assertEquals(4.0345, c.getY(), 0.002);
  }

  @Test
  void andyMarkBlueHubCenter() {
    Translation2d c = FieldTargets.hubCenter(TagFieldLayoutLoader.officialLayout(true), Alliance.Blue).orElseThrow();
    assertEquals(4.6115, c.getX(), 0.002);
    assertEquals(4.0215, c.getY(), 0.002);
  }

  @Test
  void layoutWithoutHubTagsGivesNoCenter() {
    AprilTagFieldLayout empty = new AprilTagFieldLayout(List.of(), 16.5, 8.0);
    assertTrue(FieldTargets.hubCenter(empty, Alliance.Blue).isEmpty());
  }
}
