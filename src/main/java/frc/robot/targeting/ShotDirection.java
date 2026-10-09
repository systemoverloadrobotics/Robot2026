package frc.robot.targeting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ShotConstants;

/**
 * Which end of Arc-imedes faces the hub.
 *
 * FRONT: flywheel CCW (+), fuel leaves the back-right end of the hood and flies forward; the
 * robot's front faces the hub. BACK: flywheel CW (-), fuel leaves the front-right end and flies
 * backward; the robot's back faces the hub. Hood signs come from the trench presets (front -88°,
 * back +87°) and are confirmed in the lab.
 */
public enum ShotDirection {
  FRONT(Rotation2d.kZero, 1.0, -1.0, 1.0),
  BACK(Rotation2d.fromDegrees(180.0), -1.0, 1.0, -1.0);

  private final Rotation2d m_facingOffset;
  private final double m_flywheelSign;
  private final double m_hoodSign;
  private final double m_lateralSign;

  ShotDirection(Rotation2d facingOffset, double flywheelSign, double hoodSign, double lateralSign) {
    m_facingOffset = facingOffset;
    m_flywheelSign = flywheelSign;
    m_hoodSign = hoodSign;
    m_lateralSign = lateralSign;
  }

  /** Robot heading that sends the fuel along the given field firing direction. */
  public Rotation2d robotHeadingFor(Rotation2d firingDirection) {
    return firingDirection.plus(m_facingOffset);
  }

  /**
   * How far the launch point sits to the LEFT of the firing line through the robot center.
   * Firing forward, left is robot +Y; firing backward, left is robot -Y.
   */
  public double lateralOffset(Translation2d launchPoint) {
    return m_lateralSign * launchPoint.getY();
  }

  /** Turns table magnitudes into the signed command for this direction. */
  public ShotSetpoint setpoint(double hoodMagnitudeDeg, double flywheelMagnitudeFps) {
    return new ShotSetpoint(m_hoodSign * hoodMagnitudeDeg, m_flywheelSign * flywheelMagnitudeFps);
  }

  public ShotDirection other() {
    return this == FRONT ? BACK : FRONT;
  }

  /** Where the fuel leaves the hood for this shot, robot frame, meters. */
  public Translation2d launchPoint() {
    return this == FRONT ? ShotConstants.kFrontLaunchPoint : ShotConstants.kBackLaunchPoint;
  }
}
