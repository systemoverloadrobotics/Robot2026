package frc.robot.dashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * The short list of values for the Elastic dashboard, published under "Dash/" even at
 * competition (DogLog only publishes to NetworkTables off the field).
 *
 * Cheap on both networks: about 30 values at 10 Hz is ~15 KB/s on the radio, and every value is
 * already being broadcast on CAN (motor currents, PDH), so reading them adds no CAN traffic.
 */
public final class DashboardPublisher {
  private static final int kEveryNLoops = 5; // 50 Hz loop -> 10 Hz

  private final VisionSubsystem m_vision;
  private final PowerDistribution m_pdh;
  private final List<NamedDevice> m_mechanisms;
  private final List<NamedDevice> m_drive;
  private final Supplier<String> m_mode;
  private final Supplier<String> m_direction;
  private final LoopThrottle m_throttle = new LoopThrottle(kEveryNLoops);
  private final List<String> m_allNames = new ArrayList<>();
  private final boolean[] m_connected;

  /**
   * @param mechanisms motors and encoders shown with their own current
   * @param drive swerve devices; their motor currents are summed into one value
   */
  public DashboardPublisher(
      VisionSubsystem vision,
      PowerDistribution pdh,
      List<NamedDevice> mechanisms,
      List<NamedDevice> drive,
      Supplier<String> mode,
      Supplier<String> direction) {
    m_vision = vision;
    m_pdh = pdh;
    m_mechanisms = mechanisms;
    m_drive = drive;
    m_mode = mode;
    m_direction = direction;
    mechanisms.forEach(d -> m_allNames.add(d.name()));
    drive.forEach(d -> m_allNames.add(d.name()));
    m_connected = new boolean[m_allNames.size()];
  }

  /** Call every loop; publishes every 5th. */
  public void periodic() {
    if (!m_throttle.tick()) {
      return;
    }

    // Driver
    SmartDashboard.putString("Dash/Mode", m_mode.get());
    SmartDashboard.putString("Dash/ShotDirection", m_direction.get());
    SmartDashboard.putBoolean("Dash/VisionHealthy", m_vision.isHealthy());
    SmartDashboard.putBoolean("Dash/VisionHasTarget", m_vision.hasTarget());
    SmartDashboard.putNumber("Dash/VisionAgeSec", m_vision.getSecondsSinceAccepted());
    SmartDashboard.putBoolean("Dash/Aimed", ShotStatus.aimed());
    SmartDashboard.putBoolean("Dash/ReadyToFire", ShotStatus.ready());
    SmartDashboard.putString("Dash/BlockedBy", ShotStatus.blockedBy());

    // Battery
    SmartDashboard.putNumber("Dash/BatteryVolts", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean("Dash/BrownedOut", RobotController.isBrownedOut());
    SmartDashboard.putNumber("Dash/TotalAmps", m_pdh.getTotalCurrent());

    // Currents (supply side = what the battery sees)
    for (NamedDevice d : m_mechanisms) {
      if (d.device() instanceof TalonFX motor) {
        SmartDashboard.putNumber("Dash/Amps/" + d.name(), motor.getSupplyCurrent().getValueAsDouble());
      }
    }
    double driveAmps = 0.0;
    for (NamedDevice d : m_drive) {
      if (d.device() instanceof TalonFX motor) {
        driveAmps += motor.getSupplyCurrent().getValueAsDouble();
      }
    }
    SmartDashboard.putNumber("Dash/Amps/Drive total", driveAmps);

    // CAN
    CANStatus can = RobotController.getCANStatus();
    SmartDashboard.putNumber("Dash/CAN/UtilizationPct", can.percentBusUtilization * 100.0);
    SmartDashboard.putNumber("Dash/CAN/BusOffCount", can.busOffCount);
    SmartDashboard.putNumber("Dash/CAN/TxFullCount", can.txFullCount);
    SmartDashboard.putNumber("Dash/CAN/ReceiveErrors", can.receiveErrorCount);
    SmartDashboard.putNumber("Dash/CAN/TransmitErrors", can.transmitErrorCount);
    int i = 0;
    for (NamedDevice d : m_mechanisms) {
      m_connected[i++] = d.device().isConnected();
    }
    for (NamedDevice d : m_drive) {
      m_connected[i++] = d.device().isConnected();
    }
    SmartDashboard.putString("Dash/CAN/Disconnected", DeviceHealth.summarize(m_allNames, m_connected));
  }
}
