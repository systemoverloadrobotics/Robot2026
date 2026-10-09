package frc.robot.calibration;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/**
 * Publishes the calibration values under "Cal/" and applies edits typed on a dashboard.
 * An edit is any value that differs from what this class last published. Edits are ignored
 * until the robot has been enabled once: after a restart a dashboard may re-send the previous
 * session's values, which would otherwise be applied as if just typed.
 */
public class CalibrationDashboard {
  private final CalibrationState m_state;
  private final DoubleEntry m_hood;
  private final DoubleEntry m_flywheel;
  private final DoubleEntry m_manualDistance;
  private final BooleanEntry m_useManual;
  private final StringPublisher m_direction;
  private final DoublePublisher m_distance;
  private final StringPublisher m_distanceSource;
  private final BooleanPublisher m_visionFresh;
  private final DoublePublisher m_actualHood;
  private final DoublePublisher m_actualFlywheel;
  private final StringPublisher m_lastSaved;

  private boolean m_first = true;
  private double m_lastHood;
  private double m_lastFlywheel;
  private double m_lastManualDistance;
  private boolean m_lastUseManual;

  public CalibrationDashboard(CalibrationState state) {
    this(state, NetworkTableInstance.getDefault());
  }

  public CalibrationDashboard(CalibrationState state, NetworkTableInstance nt) {
    m_state = state;
    NetworkTable t = nt.getTable("Cal");
    m_hood = t.getDoubleTopic("HoodDeg").getEntry(0.0);
    m_flywheel = t.getDoubleTopic("FlywheelFps").getEntry(0.0);
    m_manualDistance = t.getDoubleTopic("ManualDistanceFt").getEntry(0.0);
    m_useManual = t.getBooleanTopic("UseManualDistance").getEntry(false);
    m_direction = t.getStringTopic("Direction").publish();
    m_distance = t.getDoubleTopic("DistanceFt").publish();
    m_distanceSource = t.getStringTopic("DistanceSource").publish();
    m_visionFresh = t.getBooleanTopic("VisionFresh").publish();
    m_actualHood = t.getDoubleTopic("ActualHoodDeg").publish();
    m_actualFlywheel = t.getDoubleTopic("ActualFlywheelFps").publish();
    m_lastSaved = t.getStringTopic("LastSaved").publish();
    m_lastSaved.set("");
  }

  public void update(
      double distanceFt,
      String source,
      boolean visionFresh,
      double actualHoodDeg,
      double actualFlywheelFps,
      boolean acceptEdits) {
    if (!m_first && acceptEdits) {
      double hood = m_hood.get();
      if (hood != m_lastHood) {
        m_state.setTypedHoodDeg(hood);
      }
      double flywheel = m_flywheel.get();
      if (flywheel != m_lastFlywheel) {
        m_state.setTypedFlywheelFps(flywheel);
      }
      double manual = m_manualDistance.get();
      if (manual != m_lastManualDistance) {
        m_state.setManualDistanceFt(manual);
      }
      boolean useManual = m_useManual.get();
      if (useManual != m_lastUseManual) {
        m_state.setUseManualDistance(useManual);
      }
    }
    m_first = false;

    m_lastHood = m_state.hoodDeg();
    m_lastFlywheel = m_state.flywheelFps();
    m_lastManualDistance = m_state.manualDistanceFt();
    m_lastUseManual = m_state.useManualDistance();
    m_hood.set(m_lastHood);
    m_flywheel.set(m_lastFlywheel);
    m_manualDistance.set(m_lastManualDistance);
    m_useManual.set(m_lastUseManual);

    m_direction.set(m_state.direction().name());
    m_distance.set(distanceFt);
    m_distanceSource.set(source);
    m_visionFresh.set(visionFresh);
    m_actualHood.set(actualHoodDeg);
    m_actualFlywheel.set(actualFlywheelFps);
  }

  public void setLastSaved(String text) {
    m_lastSaved.set(text);
  }
}
