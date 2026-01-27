package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

/**
 * ShooterSubsystem - Controls the robot's shooting mechanism.
 * 
 * <h2>Hardware Configuration:</h2>
 * <ul>
 *   <li><b>Flywheel Motor</b> - Controls ball launch velocity</li>
 *   <li><b>Hood Angle Motor</b> - Controls left/right shooting direction</li>
 *   <li><b>Hood Wheel Motor</b> - Assists with directional ball guidance</li>
 * </ul>
 * 
 * <h2>Shooting Directions:</h2>
 * <pre>
 * LEFT SHOOT MODE (Hood angle 15°-45°):
 *   - Flywheel rotates: COUNTER-CLOCKWISE
 *   - Hood wheel rotates: CLOCKWISE
 *   
 * RIGHT SHOOT MODE (Hood angle 135°-165°):
 *   - Flywheel rotates: CLOCKWISE
 *   - Hood wheel rotates: COUNTER-CLOCKWISE
 * </pre>
 * 
 * <h2>State Machine:</h2>
 * <pre>
 * IDLE → SPINNING_UP → AIMING → READY ↔ SHOOTING
 *   ↓         ↓           ↓        ↓
 *   └─────────┴───────────┴────────┴──→ FAULT
 * </pre>
 * 
 * <h2>Safety Rules:</h2>
 * <ul>
 *   <li>Hood angle is constrained to valid range for current direction</li>
 *   <li>Motor directions are enforced based on shoot direction</li>
 *   <li>Shooter NEVER feeds balls - only reports readiness to StorageSubsystem</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

    // ============================================================
    // HARDWARE
    // ============================================================
    private final TalonFX flywheelMotor;
    private final TalonFX hoodAngleMotor;
    private final TalonFX hoodWheelMotor;

    // ============================================================
    // CONTROL REQUESTS (Phoenix 6 style)
    // ============================================================
    private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage hoodAnglePositionRequest = new PositionVoltage(0);
    private final VelocityVoltage hoodWheelVelocityRequest = new VelocityVoltage(0);
    private final VoltageOut stopRequest = new VoltageOut(0);

    // ============================================================
    // STATE
    // ============================================================
    private ShooterState currentState = ShooterState.IDLE;
    private ShootDirection currentDirection = ShootDirection.RIGHT;
    private boolean isEnabled = false;
    private boolean isAutoMode = true;

    // ============================================================
    // TARGETS
    // ============================================================
    private double targetFlywheelRPM = 0;
    private double targetHoodAngle = RIGHT_HOOD_BASE_ANGLE;
    private double targetHoodWheelRPM = 0;
    private double targetDistanceMeters = 0;

    // ============================================================
    // TIMING
    // ============================================================
    private final Timer stateTimer = new Timer();
    private final Timer rpmStableTimer = new Timer();
    private final Timer angleStableTimer = new Timer();
    private double lastRPMError = Double.MAX_VALUE;
    private double lastAngleError = Double.MAX_VALUE;

    // ============================================================
    // FAULT TRACKING
    // ============================================================
    private boolean hasFault = false;
    private String faultMessage = "";

    // ============================================================
    // LOOKUP TABLE
    // ============================================================
    private final ShooterLookupTable lookupTable = new ShooterLookupTable();

    // ============================================================
    // CONSTRUCTOR
    // ============================================================

    /**
     * Creates a new ShooterSubsystem.
     */
    public ShooterSubsystem() {
        // Initialize motors
        flywheelMotor = new TalonFX(FLYWHEEL_MOTOR_ID);
        hoodAngleMotor = new TalonFX(HOOD_ANGLE_MOTOR_ID);
        hoodWheelMotor = new TalonFX(HOOD_WHEEL_MOTOR_ID);

        // Configure motors
        configureFlywheelMotor();
        configureHoodAngleMotor();
        configureHoodWheelMotor();

        // Initialize timers
        stateTimer.reset();
        rpmStableTimer.reset();
        angleStableTimer.reset();
    }

    // ============================================================
    // MOTOR CONFIGURATION
    // ============================================================

    private void configureFlywheelMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // PID slot 0 for velocity control
        config.Slot0.kP = FLYWHEEL_kP;
        config.Slot0.kI = FLYWHEEL_kI;
        config.Slot0.kD = FLYWHEEL_kD;
        config.Slot0.kV = FLYWHEEL_kV;
        config.Slot0.kS = FLYWHEEL_kS;
        
        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = FLYWHEEL_MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Current limits for safety
        config.CurrentLimits.SupplyCurrentLimit = MAX_MOTOR_CURRENT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        flywheelMotor.getConfigurator().apply(config);
    }

    private void configureHoodAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // PID slot 0 for position control
        config.Slot0.kP = HOOD_ANGLE_kP;
        config.Slot0.kI = HOOD_ANGLE_kI;
        config.Slot0.kD = HOOD_ANGLE_kD;
        
        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = HOOD_ANGLE_MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Soft limits for safety
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = angleToRotations(HOOD_ABSOLUTE_MAX);
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = angleToRotations(HOOD_ABSOLUTE_MIN);
        
        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = MAX_MOTOR_CURRENT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        hoodAngleMotor.getConfigurator().apply(config);
    }

    private void configureHoodWheelMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // PID slot 0 for velocity control
        config.Slot0.kP = HOOD_WHEEL_kP;
        config.Slot0.kI = HOOD_WHEEL_kI;
        config.Slot0.kD = HOOD_WHEEL_kD;
        config.Slot0.kV = HOOD_WHEEL_kV;
        
        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = HOOD_WHEEL_MOTOR_INVERTED 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Current limits
        config.CurrentLimits.SupplyCurrentLimit = MAX_MOTOR_CURRENT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        hoodWheelMotor.getConfigurator().apply(config);
    }

    // ============================================================
    // PERIODIC (runs every 20ms)
    // ============================================================

    @Override
    public void periodic() {
        // Always update telemetry
        logToDashboard();

        // If disabled or faulted, don't run state machine
        if (!isEnabled || hasFault) {
            return;
        }

        // Check for faults
        checkForFaults();
        if (hasFault) {
            transitionToState(ShooterState.FAULT);
            return;
        }

        // Run state machine
        runStateMachine();
        
        // Apply motor outputs
        applyMotorOutputs();
    }

    // ============================================================
    // STATE MACHINE
    // ============================================================

    private void runStateMachine() {
        switch (currentState) {
            case IDLE:
                // Do nothing, waiting for commands
                break;

            case SPINNING_UP:
                handleSpinningUp();
                break;

            case AIMING:
                handleAiming();
                break;

            case READY:
                handleReady();
                break;

            case SHOOTING:
                handleShooting();
                break;

            case FAULT:
                // Handled separately
                break;
        }
    }

    private void handleSpinningUp() {
        // Check for spinup timeout
        if (stateTimer.hasElapsed(SPINUP_TIMEOUT)) {
            triggerFault("Flywheel spinup timeout - failed to reach target RPM");
            return;
        }

        // Check if at speed with stability
        if (isAtSpeed()) {
            if (rpmStableTimer.hasElapsed(RPM_STABLE_TIME)) {
                // RPM is stable, move to aiming
                transitionToState(ShooterState.AIMING);
            }
        } else {
            // Not at speed, reset stable timer
            rpmStableTimer.reset();
            rpmStableTimer.start();
        }
    }

    private void handleAiming() {
        // Check for aiming timeout
        if (stateTimer.hasElapsed(AIMING_TIMEOUT)) {
            triggerFault("Hood aiming timeout - failed to reach target angle");
            return;
        }

        // Check if at angle with stability
        if (isAtAngle()) {
            if (angleStableTimer.hasElapsed(ANGLE_STABLE_TIME)) {
                // Angle is stable, check if still at speed
                if (isAtSpeed()) {
                    transitionToState(ShooterState.READY);
                } else {
                    // Lost speed, go back to spinning up
                    transitionToState(ShooterState.SPINNING_UP);
                }
            }
        } else {
            // Not at angle, reset stable timer
            angleStableTimer.reset();
            angleStableTimer.start();
        }
    }

    private void handleReady() {
        // Maintain readiness - check if we're still at targets
        if (!isAtSpeed() || !isAtAngle()) {
            // Lost target, go back to appropriate state
            if (!isAtSpeed()) {
                transitionToState(ShooterState.SPINNING_UP);
            } else {
                transitionToState(ShooterState.AIMING);
            }
        }
        // Otherwise, stay ready and wait for shot signal
    }

    private void handleShooting() {
        // Brief state during shot
        if (stateTimer.hasElapsed(SHOT_DURATION)) {
            // Shot complete, return to ready if still at targets
            if (isAtSpeed() && isAtAngle()) {
                transitionToState(ShooterState.READY);
            } else {
                transitionToState(ShooterState.SPINNING_UP);
            }
        }
    }

    private void transitionToState(ShooterState newState) {
        if (currentState != newState) {
            System.out.println("[Shooter] State: " + currentState + " → " + newState);
            currentState = newState;
            stateTimer.reset();
            stateTimer.start();
            
            // Reset stability timers on state change
            rpmStableTimer.reset();
            angleStableTimer.reset();
        }
    }

    // ============================================================
    // MOTOR OUTPUT
    // ============================================================

    private void applyMotorOutputs() {
        if (currentState == ShooterState.IDLE || currentState == ShooterState.FAULT) {
            // Stop all motors when idle or faulted
            flywheelMotor.setControl(stopRequest);
            hoodWheelMotor.setControl(stopRequest);
            // Keep hood angle motor in position
            return;
        }

        // Calculate motor directions based on shoot direction
        // 
        // RIGHT SHOOT MODE:
        //   - Flywheel: CLOCKWISE (positive velocity)
        //   - Hood wheel: COUNTER-CLOCKWISE (negative velocity)
        //
        // LEFT SHOOT MODE:
        //   - Flywheel: COUNTER-CLOCKWISE (negative velocity)
        //   - Hood wheel: CLOCKWISE (positive velocity)
        //
        double flywheelDirection = (currentDirection == ShootDirection.RIGHT) ? 1.0 : -1.0;
        double hoodWheelDirection = (currentDirection == ShootDirection.RIGHT) ? -1.0 : 1.0;

        // Apply flywheel velocity
        // Convert RPM to rotations per second for Phoenix 6
        double flywheelRPS = (targetFlywheelRPM / 60.0) * flywheelDirection;
        flywheelVelocityRequest.Velocity = flywheelRPS;
        flywheelMotor.setControl(flywheelVelocityRequest);

        // Apply hood wheel velocity (proportional to flywheel)
        double hoodWheelRPS = (targetHoodWheelRPM / 60.0) * hoodWheelDirection;
        hoodWheelVelocityRequest.Velocity = hoodWheelRPS;
        hoodWheelMotor.setControl(hoodWheelVelocityRequest);

        // Apply hood angle position
        double hoodRotations = angleToRotations(targetHoodAngle);
        hoodAnglePositionRequest.Position = hoodRotations;
        hoodAngleMotor.setControl(hoodAnglePositionRequest);
    }

    // ============================================================
    // CORE CONTROL API
    // ============================================================

    /**
     * Enables the shooter subsystem.
     * Must be called before any shooting operations.
     */
    public void enableShooter() {
        if (hasFault) {
            System.out.println("[Shooter] Cannot enable - fault present. Call clearFault() first.");
            return;
        }
        isEnabled = true;
        System.out.println("[Shooter] Enabled");
    }

    /**
     * Disables the shooter subsystem.
     * Stops all motors and returns to IDLE state.
     */
    public void disableShooter() {
        isEnabled = false;
        stopAllMotors();
        transitionToState(ShooterState.IDLE);
        System.out.println("[Shooter] Disabled");
    }

    /**
     * Immediately stops all shooter motors.
     */
    public void stopAllMotors() {
        targetFlywheelRPM = 0;
        targetHoodWheelRPM = 0;
        flywheelMotor.setControl(stopRequest);
        hoodWheelMotor.setControl(stopRequest);
        transitionToState(ShooterState.IDLE);
    }

    // ============================================================
    // DIRECTION & AIMING API
    // ============================================================

    /**
     * Sets the shooting direction and validates/adjusts hood angle.
     * 
     * @param direction LEFT or RIGHT
     */
    public void setShootDirection(ShootDirection direction) {
        if (!currentState.canAcceptCommands()) {
            System.out.println("[Shooter] Cannot change direction in state: " + currentState);
            return;
        }

        if (this.currentDirection != direction) {
            this.currentDirection = direction;
            
            // Adjust hood angle to be valid for new direction
            targetHoodAngle = clampAngleForDirection(targetHoodAngle, direction);
            
            System.out.println("[Shooter] Direction set to: " + direction + 
                             ", Hood angle adjusted to: " + targetHoodAngle + "°");
        }
    }

    /**
     * Sets target distance for AUTO mode.
     * Looks up RPM and angle from the lookup table.
     * 
     * @param distanceMeters Distance to target in meters
     */
    public void setTargetDistance(double distanceMeters) {
        if (!currentState.canAcceptCommands()) {
            return;
        }

        this.isAutoMode = true;
        this.targetDistanceMeters = distanceMeters;

        // Get shot parameters from lookup table
        ShotParameters params = lookupTable.getParameters(distanceMeters);

        // Calculate actual hood angle based on direction
        double baseAngle = (currentDirection == ShootDirection.LEFT) 
            ? LEFT_HOOD_BASE_ANGLE 
            : RIGHT_HOOD_BASE_ANGLE;
        double calculatedAngle = baseAngle + params.hoodAngleOffset;

        // Apply targets
        setTargetFlywheelRPM(params.flywheelRPM);
        setTargetHoodAngle(calculatedAngle);

        System.out.println("[Shooter] Auto target: " + distanceMeters + "m → " + 
                         params.flywheelRPM + " RPM, " + calculatedAngle + "°");
    }

    /**
     * Sets target flywheel RPM directly (MANUAL mode).
     * 
     * @param rpm Target RPM (will be clamped to valid range)
     */
    public void setTargetFlywheelRPM(double rpm) {
        if (!currentState.canAcceptCommands()) {
            return;
        }

        // Clamp to valid range
        this.targetFlywheelRPM = Math.max(FLYWHEEL_MIN_RPM, 
                                         Math.min(rpm, FLYWHEEL_MAX_RPM));
        
        // Set hood wheel speed proportionally
        this.targetHoodWheelRPM = Math.min(targetFlywheelRPM * HOOD_WHEEL_SPEED_RATIO, 
                                          HOOD_WHEEL_MAX_RPM);

        // Start spinning up if not already
        if (currentState == ShooterState.IDLE && isEnabled) {
            transitionToState(ShooterState.SPINNING_UP);
        }
    }

    /**
     * Sets target hood angle directly (MANUAL mode).
     * Angle will be clamped to valid range for current direction.
     * 
     * @param degrees Target angle in degrees
     */
    public void setTargetHoodAngle(double degrees) {
        if (!currentState.canAcceptCommands()) {
            return;
        }

        // Clamp to valid range for current direction
        this.targetHoodAngle = clampAngleForDirection(degrees, currentDirection);
    }

    // ============================================================
    // AUTO CONVENIENCE API
    // ============================================================

    /**
     * Prepares a shot with specified direction and distance.
     * Combines setShootDirection() and setTargetDistance().
     * 
     * @param direction Shooting direction
     * @param distanceMeters Distance to target
     */
    public void prepareShot(ShootDirection direction, double distanceMeters) {
        setShootDirection(direction);
        setTargetDistance(distanceMeters);
    }

    /**
     * Returns true if the shooter is fully prepared and ready to shoot.
     * This is the method StorageSubsystem should check before feeding.
     */
    public boolean isPrepared() {
        return isEnabled && currentState == ShooterState.READY;
    }

    /**
     * Maintains the current ready state.
     * Call this periodically to keep the shooter ready.
     */
    public void holdReady() {
        // This is mostly a semantic method - the state machine
        // automatically maintains the ready state.
        // But we can use it to prevent timeout or auto-disable.
    }

    // ============================================================
    // MANUAL CONVENIENCE API
    // ============================================================

    /**
     * Toggles between LEFT and RIGHT shoot direction.
     */
    public void toggleShootDirection() {
        setShootDirection(currentDirection.opposite());
    }

    /**
     * Increases flywheel RPM by SPINUP_INCREMENT.
     */
    public void spinUp() {
        this.isAutoMode = false;
        setTargetFlywheelRPM(targetFlywheelRPM + SPINUP_INCREMENT);
    }

    /**
     * Decreases flywheel RPM by SPINDOWN_INCREMENT.
     */
    public void spinDown() {
        this.isAutoMode = false;
        setTargetFlywheelRPM(targetFlywheelRPM - SPINDOWN_INCREMENT);
    }

    // ============================================================
    // STORAGE COORDINATION API
    // ============================================================

    /**
     * Returns true if it's safe for StorageSubsystem to feed a ball.
     * 
     * This is the ONLY method StorageSubsystem should use to determine
     * if feeding is allowed. The shooter NEVER initiates feeding.
     * 
     * @return true if shooter is ready and safe to receive a ball
     */
    public boolean isSafeToFeed() {
        return isEnabled 
            && !hasFault 
            && currentState == ShooterState.READY 
            && isAtSpeed() 
            && isAtAngle();
    }

    /**
     * Call this when StorageSubsystem begins feeding a ball.
     * Transitions to SHOOTING state briefly.
     */
    public void notifyShotStarted() {
        if (currentState == ShooterState.READY) {
            transitionToState(ShooterState.SHOOTING);
        }
    }

    // ============================================================
    // STATUS & TELEMETRY API
    // ============================================================

    /**
     * Returns true if flywheel is within RPM tolerance of target.
     */
    public boolean isAtSpeed() {
        double currentRPM = getCurrentFlywheelRPM();
        double error = Math.abs(currentRPM - targetFlywheelRPM);
        return error < RPM_TOLERANCE;
    }

    /**
     * Returns true if hood is within angle tolerance of target.
     */
    public boolean isAtAngle() {
        double currentAngle = getCurrentHoodAngle();
        double error = Math.abs(currentAngle - targetHoodAngle);
        return error < ANGLE_TOLERANCE;
    }

    /**
     * Returns true if shooter is ready to shoot.
     * Both flywheel and hood must be at target.
     */
    public boolean isReadyToShoot() {
        return currentState == ShooterState.READY;
    }

    /**
     * Returns the current state of the shooter.
     */
    public ShooterState getShooterState() {
        return currentState;
    }

    /**
     * Returns the current shoot direction.
     */
    public ShootDirection getShootDirection() {
        return currentDirection;
    }

    /**
     * Returns current flywheel RPM.
     */
    public double getCurrentFlywheelRPM() {
        // Phoenix 6 returns rotations per second, convert to RPM
        return Math.abs(flywheelMotor.getVelocity().getValueAsDouble() * 60.0);
    }

    /**
     * Returns current hood angle in degrees.
     */
    public double getCurrentHoodAngle() {
        return rotationsToAngle(hoodAngleMotor.getPosition().getValueAsDouble());
    }

    /**
     * Returns target flywheel RPM.
     */
    public double getTargetFlywheelRPM() {
        return targetFlywheelRPM;
    }

    /**
     * Returns target hood angle.
     */
    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    /**
     * Logs shooter telemetry to SmartDashboard.
     */
    public void logToDashboard() {
        SmartDashboard.putString("Shooter/State", currentState.toString());
        SmartDashboard.putString("Shooter/Direction", currentDirection.toString());
        SmartDashboard.putBoolean("Shooter/Enabled", isEnabled);
        SmartDashboard.putBoolean("Shooter/AutoMode", isAutoMode);
        
        SmartDashboard.putNumber("Shooter/Flywheel/TargetRPM", targetFlywheelRPM);
        SmartDashboard.putNumber("Shooter/Flywheel/CurrentRPM", getCurrentFlywheelRPM());
        SmartDashboard.putBoolean("Shooter/Flywheel/AtSpeed", isAtSpeed());
        
        SmartDashboard.putNumber("Shooter/Hood/TargetAngle", targetHoodAngle);
        SmartDashboard.putNumber("Shooter/Hood/CurrentAngle", getCurrentHoodAngle());
        SmartDashboard.putBoolean("Shooter/Hood/AtAngle", isAtAngle());
        
        SmartDashboard.putBoolean("Shooter/ReadyToShoot", isReadyToShoot());
        SmartDashboard.putBoolean("Shooter/SafeToFeed", isSafeToFeed());
        
        SmartDashboard.putBoolean("Shooter/HasFault", hasFault);
        SmartDashboard.putString("Shooter/FaultMessage", faultMessage);
        
        if (isAutoMode) {
            SmartDashboard.putNumber("Shooter/TargetDistance", targetDistanceMeters);
        }
    }

    // ============================================================
    // SAFETY & FAULT HANDLING
    // ============================================================

    /**
     * Returns true if a fault has been detected.
     */
    public boolean hasFault() {
        return hasFault;
    }

    /**
     * Returns the fault message if a fault is present.
     */
    public String getFaultMessage() {
        return faultMessage;
    }

    /**
     * Clears the current fault and returns to IDLE state.
     * Call this after addressing the cause of the fault.
     */
    public void clearFault() {
        if (hasFault) {
            System.out.println("[Shooter] Fault cleared: " + faultMessage);
            hasFault = false;
            faultMessage = "";
            transitionToState(ShooterState.IDLE);
        }
    }

    /**
     * Emergency stop - immediately disables shooter and stops all motors.
     */
    public void emergencyStop() {
        System.out.println("[Shooter] EMERGENCY STOP");
        isEnabled = false;
        hasFault = true;
        faultMessage = "Emergency stop activated";
        
        // Immediately stop all motors
        flywheelMotor.setControl(stopRequest);
        hoodAngleMotor.setControl(stopRequest);
        hoodWheelMotor.setControl(stopRequest);
        
        transitionToState(ShooterState.FAULT);
    }

    private void triggerFault(String message) {
        System.out.println("[Shooter] FAULT: " + message);
        hasFault = true;
        faultMessage = message;
        stopAllMotors();
        transitionToState(ShooterState.FAULT);
    }

    private void checkForFaults() {
        // Check flywheel motor current
        double flywheelCurrent = flywheelMotor.getSupplyCurrent().getValueAsDouble();
        if (flywheelCurrent > MAX_MOTOR_CURRENT) {
            triggerFault("Flywheel overcurrent: " + flywheelCurrent + "A");
            return;
        }

        // Check hood angle motor current
        double hoodCurrent = hoodAngleMotor.getSupplyCurrent().getValueAsDouble();
        if (hoodCurrent > MAX_MOTOR_CURRENT) {
            triggerFault("Hood motor overcurrent: " + hoodCurrent + "A");
            return;
        }

        // Check hood angle is within absolute limits
        double currentAngle = getCurrentHoodAngle();
        if (currentAngle < HOOD_ABSOLUTE_MIN - 5 || currentAngle > HOOD_ABSOLUTE_MAX + 5) {
            triggerFault("Hood angle out of range: " + currentAngle + "°");
            return;
        }
    }

    // ============================================================
    // HELPER METHODS
    // ============================================================

    /**
     * Clamps an angle to the valid range for the given direction.
     * 
     * @param angle Desired angle
     * @param direction Shoot direction
     * @return Clamped angle within valid range
     */
    private double clampAngleForDirection(double angle, ShootDirection direction) {
        if (direction == ShootDirection.LEFT) {
            return Math.max(LEFT_HOOD_MIN_ANGLE, Math.min(angle, LEFT_HOOD_MAX_ANGLE));
        } else {
            return Math.max(RIGHT_HOOD_MIN_ANGLE, Math.min(angle, RIGHT_HOOD_MAX_ANGLE));
        }
    }

    /**
     * Converts hood angle (degrees) to motor rotations.
     */
    private double angleToRotations(double degrees) {
        return degrees * HOOD_ANGLE_GEAR_RATIO / 360.0;
    }

    /**
     * Converts motor rotations to hood angle (degrees).
     */
    private double rotationsToAngle(double rotations) {
        return rotations * 360.0 / HOOD_ANGLE_GEAR_RATIO;
    }

    /**
     * Returns true if the given angle is valid for the given direction.
     */
    public boolean isAngleValidForDirection(double angle, ShootDirection direction) {
        if (direction == ShootDirection.LEFT) {
            return angle >= LEFT_HOOD_MIN_ANGLE && angle <= LEFT_HOOD_MAX_ANGLE;
        } else {
            return angle >= RIGHT_HOOD_MIN_ANGLE && angle <= RIGHT_HOOD_MAX_ANGLE;
        }
    }

    /**
     * Gets the lookup table for external access (e.g., for tuning).
     */
    public ShooterLookupTable getLookupTable() {
        return lookupTable;
    }

    /**
     * Resets the hood angle encoder to a known position.
     * Call this during robot initialization with the hood at a known angle.
     * 
     * @param currentAngleDegrees The actual current angle of the hood
     */
    public void resetHoodEncoder(double currentAngleDegrees) {
        hoodAngleMotor.setPosition(angleToRotations(currentAngleDegrees));
        System.out.println("[Shooter] Hood encoder reset to: " + currentAngleDegrees + "°");
    }
}
