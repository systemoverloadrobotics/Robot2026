package frc.robot.targeting;

/** A signed shooter command: hood angle on the signed scale (deg) and flywheel speed (ft/s). */
public record ShotSetpoint(double hoodDeg, double flywheelFps) {}
