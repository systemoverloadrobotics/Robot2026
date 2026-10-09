package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.targeting.ShotDirection;
import org.junit.jupiter.api.Test;

class ShotRecorderTest {
  private static final ShotRecorder.Row kRow = new ShotRecorder.Row(
      12.5, ShotDirection.BACK, 9.0, "VISION", 64.5, 64.0, -38.0, -37.6, true);

  @Test
  void csvLineMatchesHeaderColumns() {
    assertEquals(ShotRecorder.kCsvHeader.split(",").length, ShotRecorder.csvLine(kRow).split(",").length);
    assertEquals("12.50,BACK,9.00,VISION,64.5,64.0,-38.0,-37.6,true", ShotRecorder.csvLine(kRow));
  }

  @Test
  void pasteLineUsesMagnitudesInTableOrder() {
    assertEquals("{9.0, 64.5, 38.0}, // BACK: distance ft, hood deg, flywheel ft/s", ShotRecorder.pasteLine(kRow));
  }

  @Test
  void refusesWithoutTrustedDistance() {
    assertTrue(ShotRecorder.refusalReason(Double.NaN, false, true).isPresent());
    assertTrue(ShotRecorder.refusalReason(9.0, false, false).isPresent());
    assertTrue(ShotRecorder.refusalReason(9.0, true, false).isEmpty());
    assertTrue(ShotRecorder.refusalReason(9.0, false, true).isEmpty());
  }
}
