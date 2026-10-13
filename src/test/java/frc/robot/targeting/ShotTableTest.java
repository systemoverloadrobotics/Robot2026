package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class ShotTableTest {
  private static ShotTable table() {
    return new ShotTable(0.0, 90.0).put(6.0, 40.0, 30.0).put(12.0, 60.0, 50.0);
  }

  @Test
  void interpolatesBetweenRows() {
    var shot = table().lookup(9.0);
    assertEquals(50.0, shot.hoodDeg(), 1e-9);
    assertEquals(40.0, shot.flywheelFps(), 1e-9);
  }

  @Test
  void clampsBeyondEnds() {
    assertEquals(40.0, table().lookup(1.0).hoodDeg(), 1e-9);
    assertEquals(50.0, table().lookup(30.0).flywheelFps(), 1e-9);
    assertFalse(table().inRange(30.0));
    assertTrue(table().inRange(9.0));
  }

  @Test
  void singleRowGivesTheSameShotEverywhere() {
    var t = new ShotTable(0.0, 90.0).put(10.0, 88.0, 42.0);
    assertEquals(88.0, t.lookup(3.0).hoodDeg(), 1e-9);
    assertEquals(42.0, t.lookup(20.0).flywheelFps(), 1e-9);
  }

  @Test
  void hoodIsClampedToTheUsableRange() {
    var t = new ShotTable(20.0, 70.0).put(5.0, 10.0, 30.0).put(15.0, 85.0, 60.0);
    assertEquals(20.0, t.lookup(5.0).hoodDeg(), 1e-9);
    assertEquals(70.0, t.lookup(15.0).hoodDeg(), 1e-9);
  }

  @Test
  void emptyTableThrows() {
    assertThrows(IllegalStateException.class, () -> new ShotTable(0.0, 90.0).lookup(5.0));
  }

  @Test
  void fromRowsReadsEveryRow() {
    var t = ShotTable.fromRows(new double[][] {{6.0, 40.0, 30.0}, {12.0, 60.0, 50.0}}, 0.0, 90.0);
    assertEquals(50.0, t.lookup(9.0).hoodDeg(), 1e-9);
  }

  @Test
  void configuredTablesCover4To20FeetAtFeedableSpeeds() {
    for (ShotDirection dir : ShotDirection.values()) {
      var t = ShotTable.forDirection(dir);
      assertTrue(t.inRange(4.0) && t.inRange(20.0), dir + " table should span 4-20 ft");
      for (double ft = 4.0; ft <= 20.0; ft += 1.0) {
        assertTrue(t.lookup(ft).flywheelFps() >= frc.robot.Constants.ShotConstants.kMinFeedFlywheelFps,
            dir + " flywheel too slow to feed at " + ft + " ft");
      }
    }
  }
}
