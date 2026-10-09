package frc.robot.targeting;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;

/**
 * Decides when vision should SNAP the robot pose instead of blending into it. Pure, unit-tested.
 *
 * Odometry boots at (0, 0, 0°). Blending alone moves the pose only part of the way toward the
 * first vision estimate, and every estimate after that then fails VisionFilter's jump gate, so
 * vision would never correct the pose. So:
 * - the first accepted estimate snaps translation (a single tag's heading is too noisy to trust);
 * - the first accepted multi-tag estimate snaps the whole pose, heading included;
 * - after that, estimates blend.
 * If the robot is moved while disabled (or slips), estimates keep failing the jump gate. When
 * {@code requiredConsistent} jump-rejected estimates in a row agree with each other within
 * {@code consistencyMeters}, the pose snaps to them.
 */
public final class PoseSeeder {
  public enum Action { BLEND, RESET_TRANSLATION, RESET_POSE }

  private final int m_requiredConsistent;
  private final double m_consistencyMeters;
  private boolean m_translationSeeded;
  private boolean m_headingSeeded;
  private Pose2d m_anchor;
  private int m_consistentCount;

  public PoseSeeder(int requiredConsistent, double consistencyMeters) {
    m_requiredConsistent = requiredConsistent;
    m_consistencyMeters = consistencyMeters;
  }

  /** An estimate passed every gate. Returns how to apply it. */
  public Action onAccepted(int tagCount) {
    clearRejections();
    if (!m_headingSeeded && tagCount >= 2) {
      m_translationSeeded = true;
      m_headingSeeded = true;
      return Action.RESET_POSE;
    }
    if (!m_translationSeeded) {
      m_translationSeeded = true;
      return Action.RESET_TRANSLATION;
    }
    return Action.BLEND;
  }

  /**
   * An estimate failed only the jump gate. Returns a reset once enough consistent ones have
   * arrived; empty otherwise.
   */
  public Optional<Action> onJumpRejected(Pose2d estimate, int tagCount) {
    if (m_anchor == null
        || m_anchor.getTranslation().getDistance(estimate.getTranslation()) > m_consistencyMeters) {
      m_anchor = estimate;
      m_consistentCount = 1;
    } else {
      m_consistentCount++;
    }
    if (m_consistentCount < m_requiredConsistent) {
      return Optional.empty();
    }
    clearRejections();
    if (tagCount >= 2) {
      m_headingSeeded = true;
      return Optional.of(Action.RESET_POSE);
    }
    return Optional.of(Action.RESET_TRANSLATION);
  }

  /** False until vision has placed the robot at least once; the jump gate is off until then. */
  public boolean isTranslationSeeded() {
    return m_translationSeeded;
  }

  private void clearRejections() {
    m_anchor = null;
    m_consistentCount = 0;
  }
}
