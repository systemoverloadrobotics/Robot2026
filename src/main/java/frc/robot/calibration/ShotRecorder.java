package frc.robot.calibration;

import dev.doglog.DogLog;
import frc.robot.targeting.ShotDirection;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.Locale;
import java.util.Optional;

/** Saves calibration shots that scored, as shot-table rows. */
public class ShotRecorder {
  public record Row(
      double timeSec,
      ShotDirection direction,
      double distanceFt,
      String distanceSource,
      double hoodSetDeg,
      double hoodActualDeg,
      double flywheelSetFps,
      double flywheelActualFps,
      boolean visionFresh) {}

  public static final String kCsvHeader =
      "timeSec,direction,distanceFt,distanceSource,hoodSetDeg,hoodActualDeg,"
          + "flywheelSetFps,flywheelActualFps,visionFresh";

  private final Path m_csvPath;
  private boolean m_warned;

  public ShotRecorder(Path csvPath) {
    m_csvPath = csvPath;
  }

  /** Why a row can't be saved, or empty if it can. */
  public static Optional<String> refusalReason(double distanceFt, boolean useManualDistance, boolean visionFresh) {
    if (Double.isNaN(distanceFt)) {
      return Optional.of("no distance: no aim target and manual distance is off");
    }
    if (!useManualDistance && !visionFresh) {
      return Optional.of("vision not fresh: wait for tags or turn on Cal/UseManualDistance");
    }
    return Optional.empty();
  }

  public static String csvLine(Row r) {
    return String.format(Locale.US, "%.2f,%s,%.2f,%s,%.1f,%.1f,%.1f,%.1f,%b",
        r.timeSec(), r.direction().name(), r.distanceFt(), r.distanceSource(),
        r.hoodSetDeg(), r.hoodActualDeg(), r.flywheelSetFps(), r.flywheelActualFps(), r.visionFresh());
  }

  /** A row to paste into ShotConstants.kFrontShots / kBackShots. */
  public static String pasteLine(Row r) {
    return String.format(Locale.US, "{%.1f, %.1f, %.1f}, // %s: distance ft, hood deg, flywheel ft/s",
        r.distanceFt(), Math.abs(r.hoodSetDeg()), Math.abs(r.flywheelSetFps()), r.direction().name());
  }

  /** Log, append to CSV and print the paste line. Returns the text for Cal/LastSaved. */
  public String record(Row r) {
    DogLog.log("Cal/Saved/Direction", r.direction().name());
    DogLog.log("Cal/Saved/DistanceFt", r.distanceFt());
    DogLog.log("Cal/Saved/HoodDeg", r.hoodSetDeg());
    DogLog.log("Cal/Saved/FlywheelFps", r.flywheelSetFps());
    try {
      Files.createDirectories(m_csvPath.getParent());
      boolean isNew = !Files.exists(m_csvPath);
      String text = (isNew ? kCsvHeader + "\n" : "") + csvLine(r) + "\n";
      Files.writeString(m_csvPath, text, StandardCharsets.UTF_8,
          StandardOpenOption.CREATE, StandardOpenOption.APPEND);
    } catch (IOException e) {
      if (!m_warned) {
        m_warned = true;
        System.out.println("ShotRecorder: could not write " + m_csvPath + ": " + e.getMessage());
      }
    }
    String paste = pasteLine(r);
    System.out.println("SHOT " + paste);
    return paste;
  }
}
