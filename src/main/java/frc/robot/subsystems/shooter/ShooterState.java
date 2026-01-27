package frc.robot.subsystems.shooter;

/**
 * State machine states for the ShooterSubsystem.
 */
public enum ShooterState {
    /** Shooter is disabled, all motors stopped */
    IDLE,
    
    /** Flywheel is spinning up to target RPM */
    SPINNING_UP,
    
    /** Hood is moving to target angle */
    AIMING,
    
    /** Both flywheel and hood are at target, ready to shoot */
    READY,
    
    /** Currently executing a shot (brief state) */
    SHOOTING,
    
    /** A fault has been detected, shooter is disabled */
    FAULT;

    /**
     * Returns true if the shooter is in an active (non-idle, non-fault) state.
     */
    public boolean isActive() {
        return this == SPINNING_UP || this == AIMING || this == READY || this == SHOOTING;
    }

    /**
     * Returns true if the shooter can accept new commands.
     */
    public boolean canAcceptCommands() {
        return this != FAULT && this != SHOOTING;
    }
}
