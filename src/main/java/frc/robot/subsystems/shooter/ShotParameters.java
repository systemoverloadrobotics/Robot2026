package frc.robot.subsystems.shooter;

/**
 * Holds the parameters for a shot at a specific distance.
 * Used by the lookup table to map distance to flywheel RPM and hood angle offset.
 */
public class ShotParameters {
    /** Target flywheel RPM for this distance */
    public final double flywheelRPM;
    
    /** 
     * Hood angle offset from base angle (degrees).
     * The actual angle depends on shoot direction:
     * - LEFT: baseAngle (30°) + offset
     * - RIGHT: baseAngle (150°) + offset
     */
    public final double hoodAngleOffset;

    /**
     * Creates shot parameters.
     * 
     * @param flywheelRPM Target flywheel RPM
     * @param hoodAngleOffset Hood angle offset from direction base angle
     */
    public ShotParameters(double flywheelRPM, double hoodAngleOffset) {
        this.flywheelRPM = flywheelRPM;
        this.hoodAngleOffset = hoodAngleOffset;
    }

    /**
     * Linearly interpolates between two ShotParameters.
     * 
     * @param a First parameters
     * @param b Second parameters
     * @param t Interpolation factor (0.0 = a, 1.0 = b)
     * @return Interpolated parameters
     */
    public static ShotParameters interpolate(ShotParameters a, ShotParameters b, double t) {
        double rpm = a.flywheelRPM + (b.flywheelRPM - a.flywheelRPM) * t;
        double angle = a.hoodAngleOffset + (b.hoodAngleOffset - a.hoodAngleOffset) * t;
        return new ShotParameters(rpm, angle);
    }

    @Override
    public String toString() {
        return String.format("ShotParams[RPM=%.0f, AngleOffset=%.1f°]", flywheelRPM, hoodAngleOffset);
    }
}
