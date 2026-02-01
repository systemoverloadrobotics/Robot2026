package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Climb.*;

/**
 * Subsystem for controlling the robot's climbing mechanism.
 *
 * The climb subsystem uses a single TalonFX motor with closed-loop PID position control
 * to extend, retract, and hold the climbing arm at specific positions. 
 */
public class Climb extends SubsystemBase {
  /** TalonFX motor controller for the climb mechanism. */
  private final TalonFX climbMotor;

  /** Position control request object for closed-loop PID control. */
  private final PositionVoltage climbPosReq;

  /** CAN bus instance for motor communication. */
  private final CANBus kCANBus = new CANBus(CANBUS_RIO);

  /**
   * Creates a new Climb subsystem.
   *
   * Initializes the TalonFX motor controller with PID gains, current limits,
   * brake mode, and feedback configuration from Constants.
   */
  public Climb() {
    climbMotor = new TalonFX(MOTOR_ID, kCANBus);
    climbPosReq = new PositionVoltage(0);
    climbMotor.getConfigurator().apply(getTalonFXConfiguration());
  }
 
  /**
   * Creates a command that extends the climb arm to its maximum position.
   * This is a one-shot command that sets the target position and returns immediately.
   * The motor's PID controller will move the arm to MAX_EXTN_POSITION.
   */
  public Command extendArm() {
    return runOnce(() -> setClimbPosition(MAX_EXTN_POSITION))
        .withName("extendArm");
  }

  /**
   * Creates a command that retracts the climb arm to its minimum position.
   */
  public Command retractArm() {
    return runOnce(() -> setClimbPosition(MIN_EXTN_POSITION))
        .withName("retractArm");
  }
   /**
   * Creates a command that moves the climb to the default hold position.
   */
  public Command climbAndHold() {
    return climbAndHold(HOLD_POSITION);
  }
  /**
   * Creates a command that moves the climb to a target position and holds it.
   */
  public Command climbAndHold(double position) {
    return run(() -> setClimbPosition(position))
        .withName("ClimbAndHold");
  }

  /**
   * Sets the climb motor to a target position.
   */
  public void setClimbPosition(double position) {
    climbMotor.setControl(climbPosReq.withPosition(position));
  }

  /**
   * Returns the current climb position from the motor's internal encoder.
   */
  public double getClimbPosition() {
    return climbMotor.getPosition().getValueAsDouble();
  }

  /**
   * Checks if the climb is within tolerance of a target position.
   */
  public boolean isAtPosition(double position) {
    double currentPosition = climbMotor.getPosition().getValueAsDouble();
    return Math.abs(currentPosition - position) <= POSITION_TOLERANCE;
  }

  /**
   * Checks if the climb arm is fully extended.
   */
  public boolean isExtended() {
    return isAtPosition(MAX_EXTN_POSITION);
  }

  /**
   * Checks if the climb arm is fully retracted.
   */
  public boolean isRetracted() {
    return isAtPosition(MIN_EXTN_POSITION);
  }

  /**
   * Checks if the climb arm is at the hold position.
   */
  public boolean isAtHoldPosition() {
    return isAtPosition(HOLD_POSITION);
  }

  /**
   * Creates the PID configuration for position control (Slot 0).
   */
  private Slot0Configs getSlot0Configs() {
    return new Slot0Configs()
        .withKP(KP)
        .withKI(KI)
        .withKD(KD);
  }

  /**
   * Creates the motor output configuration.
   * Configures counter-clockwise as positive direction and brake mode
   * to hold position when not actively driven.
   */
  private MotorOutputConfigs getMotorOutput() {
    return new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Creates the current limiting configuration.
   * Enables supply current limiting to protect the motor during
   * high-load climbing operations.
   */
  private CurrentLimitsConfigs getCurrentLimits() {
    return new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  /**
   * Creates the feedback sensor configuration.
   *
   * Configures the rotor sensor as the feedback source with the
   * appropriate gear ratio for position calculations.
   */
  private FeedbackConfigs getFeedbackConfigs() {
    return new FeedbackConfigs()
        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
  }

  /**
   * Combines all configuration objects into a complete TalonFX configuration.
   */
  private TalonFXConfiguration getTalonFXConfiguration() {
    return new TalonFXConfiguration()
        .withSlot0(getSlot0Configs())
        .withMotorOutput(getMotorOutput())
        .withFeedback(getFeedbackConfigs())
        .withCurrentLimits(getCurrentLimits());
  }
 

  @Override
  public void periodic() {
    // Called once per scheduler run (~20ms)
  }

  @Override
  public void simulationPeriodic() {
    // Called once per scheduler run during simulation
  }
}
