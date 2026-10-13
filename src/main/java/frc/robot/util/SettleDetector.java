package frc.robot.util;

/**
 * "Within tolerance" and "finished" are not the same condition.
 *
 * On the XRP prototype a turn reported success at 0.6 degrees of error and the
 * robot then coasted about 14 degrees past the target, because the motors were
 * cut while it was still rotating. Separately, a single lucky loop right after
 * the heading source changed ended an alignment with 13.5 degrees of real error
 * still present.
 *
 * Both are fixed by the same rule: the robot must be both close enough AND
 * essentially stopped, for several consecutive loops. This matters MORE on a
 * heavier robot, not less - more inertia, more coast.
 */
public class SettleDetector {

  private final int m_requiredLoops;
  private int m_consecutiveLoops;

  public SettleDetector(int requiredLoops) {
    m_requiredLoops = requiredLoops;
  }

  /** @return true once the settle condition has held for the required loops. */
  public boolean update(boolean withinTolerance, boolean nearlyStopped) {
    m_consecutiveLoops = (withinTolerance && nearlyStopped) ? m_consecutiveLoops + 1 : 0;
    return m_consecutiveLoops >= m_requiredLoops;
  }

  public void reset() {
    m_consecutiveLoops = 0;
  }

  public int consecutiveLoops() {
    return m_consecutiveLoops;
  }
}
