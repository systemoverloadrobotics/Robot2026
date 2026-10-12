package frc.robot.dashboard;

import com.ctre.phoenix6.hardware.ParentDevice;

/** A CAN device with a dashboard name, e.g. "Flywheel motor 1 (13)". */
public record NamedDevice(String name, ParentDevice device) {
  public static NamedDevice of(String label, ParentDevice device) {
    return new NamedDevice(label + " (" + device.getDeviceID() + ")", device);
  }
}
