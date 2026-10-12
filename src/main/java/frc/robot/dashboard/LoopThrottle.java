package frc.robot.dashboard;

/** True on the first loop and then every Nth loop, to publish at a lower rate than 50 Hz. */
public final class LoopThrottle {
  private final int m_every;
  private int m_count;

  public LoopThrottle(int every) {
    m_every = every;
  }

  public boolean tick() {
    boolean fire = m_count == 0;
    m_count = (m_count + 1) % m_every;
    return fire;
  }
}
