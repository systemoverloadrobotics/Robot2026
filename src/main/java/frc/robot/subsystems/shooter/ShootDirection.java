package frc.robot.subsystems.shooter;

/**
 * Enum representing the two shooting directions.
 * 
 * LEFT: Hood points left (15°-45°), flywheel spins counter-clockwise
 * RIGHT: Hood points right (135°-165°), flywheel spins clockwise
 */
public enum ShootDirection {
    LEFT,
    RIGHT;

    /**
     * Returns the opposite direction.
     */
    public ShootDirection opposite() {
        return this == LEFT ? RIGHT : LEFT;
    }
}
