package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraConfig;
import frc.robot.targeting.FieldTargets;
import frc.robot.targeting.PoseSeeder;
import frc.robot.targeting.TagFieldLayoutLoader;
import frc.robot.targeting.VisionFilter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.TreeSet;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The only class that talks to PhotonVision.
 *
 * Two jobs, kept separate on purpose:
 *
 * 1. LOCALIZATION (continuous, every loop): turn the tags visible to EVERY
 *    camera into field poses and fuse each one into the drivetrain's pose
 *    estimator. Nothing triggers this; it simply always runs.
 *
 * 2. RAW QUERIES (on demand): which tags are visible, what is the live bearing
 *    to one of them. Used ONLY to corroborate a targeting solution, never to
 *    steer the robot. Control always runs on the fused pose - the XRP
 *    prototype hard-switched control between vision and odometry, and the
 *    discontinuity at each switch was a real, repeatable bug.
 *
 * MULTIPLE CAMERAS: one PhotonCamera + PhotonPoseEstimator per enabled entry in
 * VisionConstants.kCameras, with no front/back special cases - ported from the
 * XRP two-camera setup. Each camera's frames are fused independently at their
 * own capture timestamps; the Kalman filter is what combines them.
 */
public class VisionSubsystem extends SubsystemBase {

  /** A camera, the estimator that knows where it's mounted, and its own health. */
  private static final class CameraEstimator {
    final CameraConfig config;
    final PhotonCamera camera;
    final PhotonPoseEstimator estimator;
    final String logPrefix;

    PhotonPipelineResult latestResult;
    double lastResultTimestamp = -1.0;
    double lastCaptureTimestamp = -1.0;
    boolean warnedNoCamera;
    boolean warnedNoCoprocMultiTag;
    boolean warnedNoCalibration;

    CameraEstimator(CameraConfig config, AprilTagFieldLayout layout) {
      this.config = config;
      this.camera = new PhotonCamera(config.name());
      this.estimator = new PhotonPoseEstimator(layout, config.robotToCamera());
      this.logPrefix = "Vision/" + config.name() + "/";
    }

    boolean isHealthy(double now) {
      return lastResultTimestamp >= 0
          && now - lastResultTimestamp < VisionConstants.kCameraTimeoutSeconds;
    }

    /** Newest frame, but only while the camera is healthy - see hasTarget(). */
    Optional<PhotonPipelineResult> liveResult(double now) {
      return isHealthy(now) && latestResult != null && latestResult.hasTargets()
          ? Optional.of(latestResult)
          : Optional.empty();
    }
  }

  /** A pose solve plus the numbers VisionFilter judges it by. */
  private record Solve(
      EstimatedRobotPose estimate, int tagCount, double worstAmbiguity, double avgDistance) {}

  private final TagFieldLayoutLoader.LoadResult m_loadResult = TagFieldLayoutLoader.load();
  private final AprilTagFieldLayout m_layout = m_loadResult.layout();
  private final List<CameraEstimator> m_cameras = new ArrayList<>();
  private final VisionSwerveDrivetrain m_drivetrain;

  private double m_lastAcceptedTimestamp = -1.0;
  private Pose2d m_lastAcceptedPose;
  private final PoseSeeder m_seeder = new PoseSeeder(
      VisionConstants.kJumpRecoveryFrames, VisionConstants.kJumpRecoveryConsistencyMeters);
  private double m_firstEnabledTimestamp = -1.0;
  private int m_rejectedCount;

  public VisionSubsystem(VisionSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    for (CameraConfig config : VisionConstants.kCameras) {
      if (config.enabled()) {
        m_cameras.add(new CameraEstimator(config, m_layout));
      }
    }

    System.out.printf(
        "Vision: %d camera(s) %s, layout=%s, multi-tag solved on %s%s%n",
        m_cameras.size(),
        m_cameras.stream().map(c -> c.config.name()).toList(),
        FieldConstants.kUsePracticeLayout ? "PRACTICE" : "OFFICIAL",
        FieldConstants.kUsePracticeLayout ? "RIO (our layout)" : "COPROCESSOR",
        m_loadResult.usedFallback()
            ? " (LAYOUT FALLBACK - targeting and pose fusion disabled; mapping still works)"
            : "");
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    for (CameraEstimator cam : m_cameras) {
      // Drain every frame queued since the last loop. Each one is a separate
      // measurement at its own capture time, so all of them are worth fusing.
      List<PhotonPipelineResult> results = cam.camera.getAllUnreadResults();
      if (!results.isEmpty()) {
        cam.latestResult = results.get(results.size() - 1);
        cam.lastResultTimestamp = now;
      }
      for (PhotonPipelineResult result : results) {
        processResult(cam, result);
      }
    }

    checkCameraHealth(now);
    logTelemetry(now);
  }

  private void processResult(CameraEstimator cam, PhotonPipelineResult result) {
    // Fail closed, same as getTargetPosition(): a fallback layout holds the
    // OFFICIAL field's tag positions, which are wrong for the practice space.
    // Fusing them would drag the pose meters away - including during a mapping
    // session, which starts with practice-layout.json deliberately removed and
    // so always runs on the fallback. Odometry alone is correct there.
    if (m_loadResult.usedFallback()) {
      return;
    }
    Optional<Solve> maybeSolve = solve(cam, result);
    if (maybeSolve.isEmpty()) {
      return;
    }
    Solve solve = maybeSolve.get();
    Pose3d pose = solve.estimate().estimatedPose;

    VisionFilter.Decision decision = VisionFilter.evaluate(
        pose, solve.tagCount(), solve.worstAmbiguity(), solve.avgDistance(),
        m_drivetrain.getRobotPose(), m_seeder.isTranslationSeeded(),
        !FieldConstants.kUsePracticeLayout);

    // Logged per camera, accepted or not - rejections are as useful as
    // acceptances when checking a pose against a tape measure, and comparing
    // the two cameras is how a wrong robotToCamera shows itself.
    DogLog.log(cam.logPrefix + "EstimatedPose", pose.toPose2d());
    DogLog.log(cam.logPrefix + "EstimatedPoseZ", pose.getZ());
    DogLog.log(cam.logPrefix + "Strategy", solve.estimate().strategy.name());
    DogLog.log(cam.logPrefix + "TagCount", solve.tagCount());
    DogLog.log(cam.logPrefix + "AvgTagDistance", solve.avgDistance());
    DogLog.log(cam.logPrefix + "Accepted", decision.accept());
    // Timing and inputs for AdvantageScope: when the frame was taken, how late it reached the
    // RIO, which tags it used, and how much the filter trusts it.
    double captureTs = result.getTimestampSeconds();
    cam.lastCaptureTimestamp = Math.max(cam.lastCaptureTimestamp, captureTs);
    DogLog.log(cam.logPrefix + "CaptureTimestamp", captureTs);
    DogLog.log(cam.logPrefix + "LatencySec", Timer.getFPGATimestamp() - captureTs);
    DogLog.log(cam.logPrefix + "TagPoses", solve.estimate().targetsUsed.stream()
        .map(t -> m_layout.getTagPose(t.getFiducialId()))
        .flatMap(Optional::stream)
        .toArray(Pose3d[]::new));
    if (decision.stdDevs() != null) {
      DogLog.log(cam.logPrefix + "StdDevs", decision.stdDevs().getData());
    }

    if (!decision.accept()) {
      m_rejectedCount++;
      DogLog.log(cam.logPrefix + "LastRejectReason", decision.reason());
      DogLog.log("Vision/LastRejectReason", cam.config.name() + ": " + decision.reason());
      if (decision.rejectedForJump()) {
        Optional<PoseSeeder.Action> reset = m_seeder.onJumpRejected(pose.toPose2d(), solve.tagCount());
        if (reset.isPresent()) {
          apply(cam, reset.get(), pose.toPose2d(), solve, decision);
          markAccepted(pose.toPose2d(), solve.estimate().timestampSeconds);
        }
      }
      return;
    }

    apply(cam, m_seeder.onAccepted(solve.tagCount()), pose.toPose2d(), solve, decision);
    markAccepted(pose.toPose2d(), solve.estimate().timestampSeconds);
  }

  /**
   * Snap or blend. A snap is needed at boot (odometry starts at the origin) and after the robot
   * was moved while vision kept rejecting jumps; see PoseSeeder.
   */
  private void apply(
      CameraEstimator cam, PoseSeeder.Action action, Pose2d pose, Solve solve,
      VisionFilter.Decision decision) {
    switch (action) {
      case RESET_POSE -> {
        m_drivetrain.resetPose(pose);
        System.out.println("Vision: pose set from " + solve.tagCount() + " tags: " + pose);
      }
      case RESET_TRANSLATION -> {
        m_drivetrain.resetTranslation(pose.getTranslation());
        System.out.println("Vision: position set from 1 tag: " + pose.getTranslation());
      }
      // CommandSwerveDrivetrain's override already converts this FPGA timestamp
      // into Phoenix's timebase, and rejects it if it falls outside the
      // estimator's history buffer.
      case BLEND -> m_drivetrain.addVisionMeasurement(
          pose, solve.estimate().timestampSeconds, decision.stdDevs());
    }
    DogLog.log("Vision/LastAction", action.name());
    DogLog.log("Vision/LastSource",
        cam.config.name() + " " + action.name() + " (" + solve.tagCount() + " tags)");
  }

  private void markAccepted(Pose2d pose, double timestampSeconds) {
    double ts = timestampSeconds;
    if (ts >= m_lastAcceptedTimestamp) {
      m_lastAcceptedPose = pose;
    }
    // Two cameras' frames arrive interleaved, so keep the newest capture time
    // rather than whichever was processed last.
    m_lastAcceptedTimestamp = Math.max(m_lastAcceptedTimestamp, ts);
  }

  /**
   * Turns one frame into a robot pose. Strategy order, from the XRP:
   *
   *   1. Multi-tag, 2+ layout tags visible. Solved on the coprocessor for the
   *      official field; on the RIO for the practice layout (see below). If
   *      the coprocessor sends no multi-tag result, solve on the RIO instead.
   *   2. Otherwise the single least-ambiguous layout tag.
   *
   * WHY THE PRACTICE LAYOUT SOLVES MULTI-TAG ON THE RIO: coprocessor multi-tag
   * PnP is solved against the layout uploaded to PhotonVision's web UI, NOT the
   * one this code loaded - PhotonPoseEstimator only applies our layout's origin
   * on that path. With a hand-placed practice layout and the stock PhotonVision
   * config, it would report poses from where those tag IDs sit on the REAL
   * field: confidently wrong, and trusted 4x more than single-tag. The RIO
   * solve (estimateRioMultiTagPose) uses OUR layout's tag positions, so it is
   * correct by construction. The previous single-camera code fell back to
   * single-tag here instead; this keeps multi-tag accuracy without the risk.
   */
  private Optional<Solve> solve(CameraEstimator cam, PhotonPipelineResult result) {
    if (!result.hasTargets()) {
      return Optional.empty();
    }

    // Only tags that are actually in our layout count.
    List<PhotonTrackedTarget> layoutTargets = result.getTargets().stream()
        .filter(t -> m_layout.getTagPose(t.getFiducialId()).isPresent())
        .toList();
    if (layoutTargets.isEmpty()) {
      return Optional.empty();
    }

    if (layoutTargets.size() >= 2) {
      Optional<EstimatedRobotPose> multi = Optional.empty();
      if (!FieldConstants.kUsePracticeLayout) {
        multi = cam.estimator.estimateCoprocMultiTagPose(result);
        if (multi.isEmpty() && !cam.warnedNoCoprocMultiTag) {
          cam.warnedNoCoprocMultiTag = true;
          System.out.println("Vision: " + cam.config.name() + " sees 2+ tags but sent no "
              + "multi-tag result - is the layout uploaded to PhotonVision and \"Do "
              + "Multi-Target\" on? Solving multi-tag on the RIO instead.");
        }
      }
      if (multi.isEmpty()) {
        multi = solveMultiTagOnRio(cam, result);
      }
      if (multi.isPresent()) {
        double worstAmbiguity = layoutTargets.stream()
            .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity).max().orElse(0.0);
        double avgDistance = layoutTargets.stream()
            .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
            .average().orElse(0.0);
        return Optional.of(
            new Solve(multi.get(), layoutTargets.size(), worstAmbiguity, avgDistance));
      }
    }

    // Single tag, done by hand rather than with estimateLowestAmbiguityPose():
    // that method reports EVERY visible target in targetsUsed, so with two tags
    // in view a single-tag solve would be counted as multi-tag - trusted 4x
    // more and skipping the ambiguity gate. Here the count, distance, and
    // ambiguity all belong to the one tag the pose actually came from.
    PhotonTrackedTarget best = layoutTargets.stream()
        .filter(t -> t.getPoseAmbiguity() >= 0)
        .min(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity))
        .orElse(null);
    if (best == null) {
      return Optional.empty();
    }
    Pose3d robotPose = m_layout.getTagPose(best.getFiducialId()).get()
        .transformBy(best.getBestCameraToTarget().inverse())
        .transformBy(cam.config.robotToCamera().inverse());
    EstimatedRobotPose estimate = new EstimatedRobotPose(
        robotPose, result.getTimestampSeconds(), List.of(best), PoseStrategy.LOWEST_AMBIGUITY);
    return Optional.of(new Solve(estimate, 1, best.getPoseAmbiguity(),
        best.getBestCameraToTarget().getTranslation().getNorm()));
  }

  /** Multi-tag PnP on the RIO against our layout. Needs the camera's calibration. */
  private Optional<EstimatedRobotPose> solveMultiTagOnRio(
      CameraEstimator cam, PhotonPipelineResult result) {
    var cameraMatrix = cam.camera.getCameraMatrix();
    var distCoeffs = cam.camera.getDistCoeffs();
    if (cameraMatrix.isEmpty() || distCoeffs.isEmpty()) {
      if (!cam.warnedNoCalibration) {
        cam.warnedNoCalibration = true;
        System.out.println("Vision: " + cam.config.name() + " has published no calibration - "
            + "cannot solve multi-tag on the RIO; using single-tag solves. Calibrate this "
            + "camera in PhotonVision at the resolution the pipeline runs.");
      }
      return Optional.empty();
    }
    return cam.estimator.estimateRioMultiTagPose(result, cameraMatrix.get(), distCoeffs.get());
  }

  /**
   * A PhotonCamera constructed with a name that does not match the
   * PhotonVision configuration does not throw - it silently yields zero
   * results forever, which is indistinguishable from "no tag in view". This
   * is the loud startup check that makes that case debuggable, per camera.
   */
  private void checkCameraHealth(double now) {
    if (!DriverStation.isEnabled()) {
      return;
    }
    if (m_firstEnabledTimestamp < 0) {
      m_firstEnabledTimestamp = now;
    }
    if (now - m_firstEnabledTimestamp <= VisionConstants.kCameraTimeoutSeconds) {
      return;
    }
    for (CameraEstimator cam : m_cameras) {
      if (!cam.warnedNoCamera && cam.lastResultTimestamp < 0) {
        cam.warnedNoCamera = true;
        DriverStation.reportError(
            "No result from PhotonVision camera '" + cam.config.name() + "' after "
                + VisionConstants.kCameraTimeoutSeconds + "s. Check that the name matches the "
                + "PhotonVision web UI exactly, and that the coprocessor is up. If this camera "
                + "is not mounted, set enabled=false in VisionConstants.",
            false);
      }
    }
  }

  private void logTelemetry(double now) {
    for (CameraEstimator cam : m_cameras) {
      DogLog.log(cam.logPrefix + "Healthy", cam.isHealthy(now));
      DogLog.log(cam.logPrefix + "HasTarget", cam.liveResult(now).isPresent());
      DogLog.log(cam.logPrefix + "FrameAgeSec",
          cam.lastCaptureTimestamp < 0 ? -1.0 : now - cam.lastCaptureTimestamp);
    }
    DogLog.log("Vision/Healthy", isHealthy());
    DogLog.log("Vision/HasTarget", hasTarget());
    DogLog.log("Vision/VisibleTagIds", getVisibleTagIds());
    DogLog.log("Vision/RejectedCount", m_rejectedCount);
    DogLog.log("Vision/SecondsSinceAccepted",
        m_lastAcceptedTimestamp < 0 ? -1.0 : getSecondsSinceAccepted());
    Pose2d pose = m_drivetrain.getRobotPose();
    DogLog.log("Drivetrain/Pose", pose);
    DogLog.log("Drivetrain/RobotX", pose.getX());
    DogLog.log("Drivetrain/RobotY", pose.getY());
    DogLog.log("Drivetrain/RobotHeadingDeg", pose.getRotation().getDegrees());
    DogLog.log("Drivetrain/YawRateDegPerSec", m_drivetrain.getYawRateDegPerSec());
  }

  public AprilTagFieldLayout getLayout() {
    return m_layout;
  }

  /**
   * Where a tag sits on the field, flattened to 2D. Empty if not in the
   * layout - OR if the practice layout was requested but a fallback to the
   * official layout occurred. Without this second check, targeting would stay
   * armed and happily find, say, tag 16 at its official FIELD position - a
   * real coordinate several meters away, in a practice room where that tag
   * does not exist - and drive toward it at up to kMaxApproachSpeedMps. Fail
   * closed instead: commands already treat an empty target as "not in the
   * layout" and finish with zero motion, so this routes into tested behavior.
   */
  public Optional<Translation2d> getTargetPosition(int tagId) {
    if (m_loadResult.usedFallback()) {
      return Optional.empty();
    }
    return m_layout.getTagPose(tagId).map(pose -> pose.toPose2d().getTranslation());
  }

  /**
   * True only when some camera has a recent, healthy frame with targets. The
   * health check is required, not optional: a camera's latest frame is set
   * once per frame and never cleared, so without it a frozen camera or
   * coprocessor would leave hasTarget() returning true forever off a dead
   * frame. That would let AimAtTagCommand and DriveToTagCommand report
   * CAMERA-VERIFIED off a stale image, and - worse - would let
   * MapVisibleTagsCommand record tag positions as the CURRENT robot pose
   * composed with an OLD camera frame, silently corrupting the practice layout
   * that everything else depends on.
   */
  public boolean hasTarget() {
    double now = Timer.getFPGATimestamp();
    return m_cameras.stream().anyMatch(cam -> cam.liveResult(now).isPresent());
  }

  /** True when the named camera has a recent, healthy frame with at least one tag. */
  public boolean cameraSeesTag(String cameraName) {
    double now = Timer.getFPGATimestamp();
    return m_cameras.stream()
        .anyMatch(cam -> cam.config.name().equals(cameraName) && cam.liveResult(now).isPresent());
  }

  /** True when EVERY enabled camera is delivering frames. */
  public boolean isHealthy() {
    double now = Timer.getFPGATimestamp();
    return !m_cameras.isEmpty() && m_cameras.stream().allMatch(cam -> cam.isHealthy(now));
  }

  /** Per-camera health for the Back-button print, e.g. "front=OK back=DOWN". */
  public String describeCameraHealth() {
    double now = Timer.getFPGATimestamp();
    StringBuilder sb = new StringBuilder();
    for (CameraEstimator cam : m_cameras) {
      if (sb.length() > 0) {
        sb.append(' ');
      }
      sb.append(cam.config.name()).append('=').append(cam.isHealthy(now) ? "OK" : "DOWN");
    }
    return sb.toString();
  }

  /** Capture time (FPGA) of the newest fused vision measurement, or -1 if none yet. */
  public double getLastAcceptedTimestamp() {
    return m_lastAcceptedTimestamp;
  }

  /**
   * Seconds since vision last corrected the fused pose; infinity if it never
   * has. Measured from frame CAPTURE time, so camera latency counts against it.
   */
  public double getSecondsSinceAccepted() {
    return m_lastAcceptedTimestamp < 0
        ? Double.POSITIVE_INFINITY
        : Timer.getFPGATimestamp() - m_lastAcceptedTimestamp;
  }

  /** Every tag ID visible to any healthy camera, sorted, without duplicates. */
  public int[] getVisibleTagIds() {
    double now = Timer.getFPGATimestamp();
    TreeSet<Integer> ids = new TreeSet<>();
    for (CameraEstimator cam : m_cameras) {
      cam.liveResult(now).ifPresent(result -> {
        for (PhotonTrackedTarget target : result.getTargets()) {
          ids.add(target.getFiducialId());
        }
      });
    }
    return ids.stream().mapToInt(Integer::intValue).toArray();
  }

  /**
   * Live robot-relative bearing to a specific tag, straight from the current
   * camera frames. Used ONLY to corroborate an arrival, never to steer.
   * Empty when that tag is not currently visible to any camera.
   */
  public Optional<Rotation2d> getLiveBearingToTag(int tagId) {
    return getRobotToTagTransform(tagId)
        .map(robotToTag -> robotToTag.getTranslation().toTranslation2d().getAngle());
  }

  /**
   * Full robot-relative 3D transform to a specific visible tag, that camera's
   * mount offset included. If more than one camera sees the tag, the closest
   * view wins - tag pose error grows with range. Used by the layout mapper,
   * which needs the tag's rotation as well as its position.
   */
  public Optional<Transform3d> getRobotToTagTransform(int tagId) {
    double now = Timer.getFPGATimestamp();
    Transform3d best = null;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (CameraEstimator cam : m_cameras) {
      Optional<PhotonPipelineResult> result = cam.liveResult(now);
      if (result.isEmpty()) {
        continue;
      }
      for (PhotonTrackedTarget target : result.get().getTargets()) {
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();
        if (target.getFiducialId() == tagId && distance < bestDistance) {
          bestDistance = distance;
          best = cam.config.robotToCamera().plus(target.getBestCameraToTarget());
        }
      }
    }
    return Optional.ofNullable(best);
  }

  /**
   * What to aim at: the alliance hub's center on the official field, or the lab tag when the
   * practice layout is on. Empty when the alliance is unknown, the layout fell back, or the
   * target isn't in the layout - commands then don't rotate.
   */
  public Optional<Translation2d> getAimTarget(Optional<Alliance> alliance) {
    if (m_loadResult.usedFallback()) {
      return Optional.empty();
    }
    if (FieldConstants.kUsePracticeLayout) {
      return getTargetPosition(FieldConstants.kLabTargetTagId);
    }
    return alliance.flatMap(a -> FieldTargets.hubCenter(m_layout, a));
  }

  public boolean isVisionFresh(double maxAgeSec) {
    return getSecondsSinceAccepted() <= maxAgeSec;
  }

  /**
   * Snap the pose translation to the newest accepted vision pose (Start button). Refuses when
   * vision is older than kMaxVisionAgeToResetSec. Heading is left to the gyro.
   */
  public boolean resetTranslationFromVision() {
    if (m_lastAcceptedPose == null || !isVisionFresh(AimConstants.kMaxVisionAgeToResetSec)) {
      System.out.println("Vision reset refused: no accepted vision pose in the last "
          + AimConstants.kMaxVisionAgeToResetSec + "s");
      return false;
    }
    m_drivetrain.resetTranslation(m_lastAcceptedPose.getTranslation());
    return true;
  }
}
