package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class climb extends SubsystemBase {
    private final TalonFX climbMotor1;
    private final CANcoder climbCANCoder;
    private final CANBus kCANBus = new CANBus("rio");
    private final PositionVoltage climbPosReq;
  /** Creates a new climb. */
  public climb() {
    /** Gives ID to TalonFX and CANcoder */
    climbMotor1 = new TalonFX(Constants.Climb.ClimbMotor1, kCANBus);
    climbCANCoder = new CANcoder(Constants.Climb.kClimbCANCoder, kCANBus);
    climbPosReq = new PositionVoltage(0);

    /** Configures the TalonFX */
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKP(Constants.Climb.kP);
    slot0Configs .withKI(Constants.Climb.kI);
    slot0Configs.withKD(Constants.Climb.kD);
    
    
    var MOCclimb1 = new MotorOutputConfigs();
    MOCclimb1.Inverted = InvertedValue.CounterClockwise_Positive;
    MOCclimb1.NeutralMode = NeutralModeValue.Brake;
    TalonFXConfiguration climbConfig1 = new TalonFXConfiguration();
   climbConfig1
    .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(Constants.Climb.sensorMechansimRatio)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
    .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(Constants.Climb.supplyCurrentLimit))
    .withSlot0(slot0Configs)  // Keep your PIDF, kF, output limits, etc.
    .withMotorOutput(MOCclimb1);

    climbMotor1.getConfigurator().apply(climbConfig1);
  }

  public void setClimbPosition(double position){
    climbMotor1.setControl(climbPosReq.withPosition(position));
    
  }
    

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command climbMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean climbCondition() {
    // Query some boolean state, such as a digital sensor.
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
