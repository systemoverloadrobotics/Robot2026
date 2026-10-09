package frc.robot.calibration;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.ShotConstants;
import frc.robot.targeting.ShotDirection;
import frc.robot.targeting.ShotSetpoint;
import frc.robot.targeting.ShotTable;

/** CALIBRATION-mode values: direction, hood and flywheel magnitudes, distance source. */
public class CalibrationState {
  private final ShotTable m_front;
  private final ShotTable m_back;
  private ShotDirection m_direction = ShotDirection.FRONT;
  private double m_hoodDeg;
  private double m_flywheelFps;
  private boolean m_useManualDistance;
  private double m_manualDistanceFt = 10.0;

  public CalibrationState(ShotTable front, ShotTable back) {
    m_front = front;
    m_back = back;
    resetToTable(m_manualDistanceFt);
  }

  /** Start from the table's entry for this distance and the current direction. */
  public void resetToTable(double distanceFt) {
    ShotTable.Shot shot = table().lookup(Double.isNaN(distanceFt) ? m_manualDistanceFt : distanceFt);
    setHoodDeg(shot.hoodDeg());
    setFlywheelFps(shot.flywheelFps());
  }

  public void toggleDirection(double distanceFt) {
    m_direction = m_direction.other();
    resetToTable(distanceFt);
  }

  public void stepHood(double deltaDeg) {
    setHoodDeg(m_hoodDeg + deltaDeg);
  }

  public void stepFlywheel(double deltaFps) {
    setFlywheelFps(m_flywheelFps + deltaFps);
  }

  public void setHoodDeg(double hoodDeg) {
    m_hoodDeg = MathUtil.clamp(hoodDeg, table().hoodMinDeg(), table().hoodMaxDeg());
  }

  /** A value typed on the dashboard: always a magnitude, whatever sign was typed. */
  public void setTypedHoodDeg(double hoodDeg) {
    setHoodDeg(Math.abs(hoodDeg));
  }

  /** A value typed on the dashboard: always a magnitude, whatever sign was typed. */
  public void setTypedFlywheelFps(double flywheelFps) {
    setFlywheelFps(Math.abs(flywheelFps));
  }

  public void setFlywheelFps(double flywheelFps) {
    m_flywheelFps = MathUtil.clamp(flywheelFps, 0.0, ShotConstants.kMaxFlywheelFps);
  }

  public void setUseManualDistance(boolean useManual) {
    m_useManualDistance = useManual;
  }

  public void setManualDistanceFt(double distanceFt) {
    m_manualDistanceFt = Math.max(0.0, distanceFt);
  }

  public ShotSetpoint setpoint() {
    return m_direction.setpoint(m_hoodDeg, m_flywheelFps);
  }

  public ShotDirection direction() {
    return m_direction;
  }

  public double hoodDeg() {
    return m_hoodDeg;
  }

  public double flywheelFps() {
    return m_flywheelFps;
  }

  public boolean useManualDistance() {
    return m_useManualDistance;
  }

  public double manualDistanceFt() {
    return m_manualDistanceFt;
  }

  private ShotTable table() {
    return m_direction == ShotDirection.FRONT ? m_front : m_back;
  }
}
