package frc.robot.targeting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants.ShotConstants;

/**
 * Distance (ft) to hood angle and flywheel speed, as positive magnitudes. Linear interpolation
 * between rows, clamped to the nearest row outside them. ShotDirection applies the signs.
 */
public final class ShotTable {
  public record Shot(double hoodDeg, double flywheelFps) {}

  private final InterpolatingDoubleTreeMap m_hood = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap m_flywheel = new InterpolatingDoubleTreeMap();
  private final double m_hoodMinDeg;
  private final double m_hoodMaxDeg;
  private double m_minDistanceFt = Double.POSITIVE_INFINITY;
  private double m_maxDistanceFt = Double.NEGATIVE_INFINITY;
  private int m_rows;

  public ShotTable(double hoodMinDeg, double hoodMaxDeg) {
    m_hoodMinDeg = hoodMinDeg;
    m_hoodMaxDeg = hoodMaxDeg;
  }

  public ShotTable put(double distanceFt, double hoodDeg, double flywheelFps) {
    m_hood.put(distanceFt, hoodDeg);
    m_flywheel.put(distanceFt, flywheelFps);
    m_minDistanceFt = Math.min(m_minDistanceFt, distanceFt);
    m_maxDistanceFt = Math.max(m_maxDistanceFt, distanceFt);
    m_rows++;
    return this;
  }

  public Shot lookup(double distanceFt) {
    if (m_rows == 0) {
      throw new IllegalStateException("Shot table has no rows");
    }
    double hood = MathUtil.clamp(m_hood.get(distanceFt), m_hoodMinDeg, m_hoodMaxDeg);
    return new Shot(hood, m_flywheel.get(distanceFt));
  }

  /** False when the distance is outside the measured rows (lookup clamps). */
  public boolean inRange(double distanceFt) {
    return distanceFt >= m_minDistanceFt && distanceFt <= m_maxDistanceFt;
  }

  public double hoodMinDeg() {
    return m_hoodMinDeg;
  }

  public double hoodMaxDeg() {
    return m_hoodMaxDeg;
  }

  public static ShotTable fromRows(double[][] rows, double hoodMinDeg, double hoodMaxDeg) {
    ShotTable table = new ShotTable(hoodMinDeg, hoodMaxDeg);
    for (double[] row : rows) {
      table.put(row[0], row[1], row[2]);
    }
    return table;
  }

  public static ShotTable forDirection(ShotDirection direction) {
    return direction == ShotDirection.FRONT
        ? fromRows(ShotConstants.kFrontShots, ShotConstants.kFrontHoodMinDeg, ShotConstants.kFrontHoodMaxDeg)
        : fromRows(ShotConstants.kBackShots, ShotConstants.kBackHoodMinDeg, ShotConstants.kBackHoodMaxDeg);
  }
}
