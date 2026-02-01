package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you may modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  // Motor and position request objects
  private final TalonFX climbMotor1;
  private final PositionVoltage climbPosReq;
  private final CANBus kCANBus = new CANBus("rio");

  /** Creates a new climb subsystem */
  public Climb() {
    // Initialize the motor on the CAN bus
    climbMotor1 = new TalonFX(Constants.Climb.ClimbMotor1, kCANBus);
    climbPosReq = new PositionVoltage(0);

    // Create and configure PIDF gains (Slot 0)
    Slot0Configs slot0Configs = new Slot0Configs()
        .withKP(Constants.Climb.kP)   // Proportional gain
        .withKI(Constants.Climb.kI)   //Integral gain
        .withKD(Constants.Climb.kD);  // Derivative gain

    // Create and configure motor output settings
    MotorOutputConfigs motorOutput = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    // Create and configure current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.Climb.supplyCurrentLimit);

    // Combine all configurations into one config object
    TalonFXConfiguration config = new TalonFXConfiguration()
        .withSlot0(slot0Configs)         // Apply PIDF settings
        .withMotorOutput(motorOutput)    // Apply motor direction and brake mode
        .withCurrentLimits(currentLimits); // Apply current limits

    // Apply the complete configuration to the motor
    climbMotor1.getConfigurator().apply(config);
  }

  /** Sets the climb motor to a target position */
  public void setClimbPosition(double position) {
    // Send position request to motor using the built-in closed-loop PID
    climbMotor1.setControl(climbPosReq.withPosition(position));
  }

  /** Returns the current climb position from the built-in motor encoder in degrees */
  public Angle getClimbPosition() {
    // Get position from TalonFX's internal encoder and convert to Degrees
    return climbMotor1.getPosition().getValue();
  }

  /** Example command factory method */
  public Command climbMethodCommand() {
    return runOnce(() -> {
      /* one-time action goes here */
    });
  }

  /** Returns a boolean state of the subsystem */
  public boolean climbCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
