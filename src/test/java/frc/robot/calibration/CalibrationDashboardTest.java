package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.targeting.ShotTable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CalibrationDashboardTest {
  private NetworkTableInstance m_nt;
  private CalibrationState m_state;
  private CalibrationDashboard m_dashboard;
  private DoublePublisher m_typedHood;

  @BeforeEach
  void setUp() {
    m_nt = NetworkTableInstance.create();
    ShotTable table = new ShotTable(0.0, 90.0).put(10.0, 60.0, 40.0);
    m_state = new CalibrationState(table, table);
    m_dashboard = new CalibrationDashboard(m_state, m_nt);
    m_typedHood = m_nt.getTable("Cal").getDoubleTopic("HoodDeg").publish();
  }

  @AfterEach
  void tearDown() {
    m_nt.close();
  }

  private void update(boolean acceptEdits) {
    m_dashboard.update(10.0, "MANUAL", false, 0.0, 0.0, acceptEdits);
  }

  @Test
  void editsBeforeTheFirstEnableAreIgnored() {
    update(false);
    m_typedHood.set(30.0);
    update(false);
    assertEquals(60.0, m_state.hoodDeg(), 1e-9);
  }

  @Test
  void editsAfterEnableAreApplied() {
    update(false);
    m_typedHood.set(30.0);
    update(true);
    assertEquals(30.0, m_state.hoodDeg(), 1e-9);
  }

  @Test
  void typedNegativeHoodIsTreatedAsMagnitude() {
    update(true);
    m_typedHood.set(-30.0);
    update(true);
    assertEquals(30.0, m_state.hoodDeg(), 1e-9);
  }
}
