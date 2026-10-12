package frc.robot.dashboard;

import java.util.ArrayList;
import java.util.List;

/** Turns per-device connected flags into one short dashboard string. */
public final class DeviceHealth {
  private DeviceHealth() {}

  /** "OK" when every device is connected, else the disconnected names in order. */
  public static String summarize(List<String> names, boolean[] connected) {
    List<String> missing = new ArrayList<>();
    for (int i = 0; i < names.size(); i++) {
      if (!connected[i]) {
        missing.add(names.get(i));
      }
    }
    return missing.isEmpty() ? "OK" : String.join(", ", missing);
  }
}
