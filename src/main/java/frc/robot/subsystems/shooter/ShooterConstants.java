package frc.robot.subsystems.shooter;

/**
 * Constants for the ShooterSubsystem.
 * 
 * All hardware IDs, physical limits, PID gains, and tolerances are defined here.
 * Update these values based on your actual robot configuration.
 */
public final class ShooterConstants {

    private ShooterConstants() {
        // Prevent instantiation
    }

    // ============================================================
    // MOTOR CAN IDs
    // ============================================================
    // TODO: Update these to match your actual CAN IDs
    
    /** Flywheel motor CAN ID - controls ball launch velocity */
    public static final int FLYWHEEL_MOTOR_ID = 20;
    
    /** Hood angle motor CAN ID - controls left/right aiming direction */
    public static final int HOOD_ANGLE_MOTOR_ID = 21;
    
    /** Hood wheel motor CAN ID - assists with directional ball guidance */
    public static final int HOOD_WHEEL_MOTOR_ID = 22;

    // ============================================================
    // HOOD ANGLE LIMITS (degrees)
    // ============================================================
    // The hood angle determines which direction the ball exits.
    // These limits are SAFETY-CRITICAL and must match mechanical design.
    
    /**
     * LEFT SHOOT MODE angle range: 15° - 45°
     * Hood points to the left side of the robot
     */
    public static final double LEFT_HOOD_MIN_ANGLE = 15.0;
    public static final double LEFT_HOOD_MAX_ANGLE = 45.0;
    public static final double LEFT_HOOD_BASE_ANGLE = 30.0;  // Center of left range
    
    /**
     * RIGHT SHOOT MODE angle range: 135° - 165°
     * Hood points to the right side of the robot
     */
    public static final double RIGHT_HOOD_MIN_ANGLE = 135.0;
    public static final double RIGHT_HOOD_MAX_ANGLE = 165.0;
    public static final double RIGHT_HOOD_BASE_ANGLE = 150.0;  // Center of right range

    /** Absolute minimum hood angle (mechanical limit) */
    public static final double HOOD_ABSOLUTE_MIN = 10.0;
    
    /** Absolute maximum hood angle (mechanical limit) */
    public static final double HOOD_ABSOLUTE_MAX = 170.0;

    // ============================================================
    // FLYWHEEL LIMITS
    // ============================================================
    
    /** Minimum flywheel RPM (below this, motor may not spin reliably) */
    public static final double FLYWHEEL_MIN_RPM = 500.0;
    
    /** Maximum flywheel RPM (motor/mechanical limit) */
    public static final double FLYWHEEL_MAX_RPM = 6000.0;
    
    /** Default idle RPM when shooter is enabled but not targeting */
    public static final double FLYWHEEL_IDLE_RPM = 1000.0;

    // ============================================================
    // HOOD WHEEL LIMITS
    // ============================================================
    
    /** Hood wheel speed as percentage of flywheel speed */
    public static final double HOOD_WHEEL_SPEED_RATIO = 0.5;
    
    /** Maximum hood wheel RPM */
    public static final double HOOD_WHEEL_MAX_RPM = 3000.0;

    // ============================================================
    // TOLERANCES
    // ============================================================
    // These determine when the shooter is considered "at target"
    
    /** RPM tolerance for isAtSpeed() check */
    public static final double RPM_TOLERANCE = 75.0;
    
    /** Angle tolerance for isAtAngle() check (degrees) */
    public static final double ANGLE_TOLERANCE = 1.5;
    
    /** How long RPM must be stable before considered ready (seconds) */
    public static final double RPM_STABLE_TIME = 0.1;
    
    /** How long angle must be stable before considered ready (seconds) */
    public static final double ANGLE_STABLE_TIME = 0.05;

    // ============================================================
    // FLYWHEEL PID GAINS
    // ============================================================
    // Velocity PID + Feedforward for flywheel control
    // TODO: Tune these values on the actual robot!
    
    public static final double FLYWHEEL_kP = 0.1;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.0;
    
    /** Feedforward: Volts per RPM (kV) */
    public static final double FLYWHEEL_kV = 0.0019;  // ~12V / 6000RPM
    
    /** Feedforward: Volts to overcome static friction (kS) */
    public static final double FLYWHEEL_kS = 0.1;

    // ============================================================
    // HOOD ANGLE PID GAINS
    // ============================================================
    // Position PID for hood angle control
    // TODO: Tune these values on the actual robot!
    
    public static final double HOOD_ANGLE_kP = 0.05;
    public static final double HOOD_ANGLE_kI = 0.0;
    public static final double HOOD_ANGLE_kD = 0.001;
    
    /** Maximum hood motor output (0-1) for safety */
    public static final double HOOD_MAX_OUTPUT = 0.5;

    // ============================================================
    // HOOD WHEEL PID GAINS
    // ============================================================
    // Velocity control for hood wheel (simpler than flywheel)
    
    public static final double HOOD_WHEEL_kP = 0.05;
    public static final double HOOD_WHEEL_kI = 0.0;
    public static final double HOOD_WHEEL_kD = 0.0;
    public static final double HOOD_WHEEL_kV = 0.002;

    // ============================================================
    // ENCODER CONFIGURATION
    // ============================================================
    
    /** Gear ratio from flywheel motor to flywheel (motor rotations per flywheel rotation) */
    public static final double FLYWHEEL_GEAR_RATIO = 1.0;  // 1:1 direct drive
    
    /** Gear ratio from hood motor to hood angle mechanism */
    public static final double HOOD_ANGLE_GEAR_RATIO = 50.0;  // 50:1 reduction
    
    /** Hood angle encoder counts per degree (after gear ratio) */
    public static final double HOOD_DEGREES_PER_ROTATION = 360.0 / HOOD_ANGLE_GEAR_RATIO;

    // ============================================================
    // FAULT DETECTION
    // ============================================================
    
    /** Maximum current before fault (amps) */
    public static final double MAX_MOTOR_CURRENT = 40.0;
    
    /** Timeout for reaching target RPM (seconds) */
    public static final double SPINUP_TIMEOUT = 3.0;
    
    /** Timeout for reaching target angle (seconds) */
    public static final double AIMING_TIMEOUT = 2.0;
    
    /** Minimum time in SHOOTING state before returning to READY (seconds) */
    public static final double SHOT_DURATION = 0.25;

    // ============================================================
    // MOTOR INVERSION
    // ============================================================
    // These define the "positive" direction for each motor.
    // Actual rotation direction is handled in code based on ShootDirection.
    
    /** If true, flywheel motor positive = clockwise when viewed from front */
    public static final boolean FLYWHEEL_MOTOR_INVERTED = false;
    
    /** If true, hood angle motor positive = increasing angle */
    public static final boolean HOOD_ANGLE_MOTOR_INVERTED = false;
    
    /** If true, hood wheel motor positive = clockwise when viewed from front */
    public static final boolean HOOD_WHEEL_MOTOR_INVERTED = false;

    // ============================================================
    // SPIN UP/DOWN INCREMENTS (for manual control)
    // ============================================================
    
    /** RPM increment for spinUp() command */
    public static final double SPINUP_INCREMENT = 250.0;
    
    /** RPM decrement for spinDown() command */
    public static final double SPINDOWN_INCREMENT = 250.0;
}
