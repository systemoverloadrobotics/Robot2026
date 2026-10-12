package frc.robot.dashboard;

/**
 * Latest aim/shot state, set by the shooting commands and read by DashboardPublisher. Commands
 * run on the main robot thread, so plain static fields are enough.
 */
public final class ShotStatus {
  private static boolean s_aimed;
  private static boolean s_ready;
  private static String s_blockedBy = "IDLE";

  private ShotStatus() {}

  public static void setAimed(boolean aimed) {
    s_aimed = aimed;
  }

  public static void setReady(boolean ready, String blockedBy) {
    s_ready = ready;
    s_blockedBy = blockedBy;
  }

  /** No shooting command running. */
  public static void clear() {
    s_aimed = false;
    s_ready = false;
    s_blockedBy = "IDLE";
  }

  public static boolean aimed() {
    return s_aimed;
  }

  public static boolean ready() {
    return s_ready;
  }

  public static String blockedBy() {
    return s_blockedBy;
  }
}
