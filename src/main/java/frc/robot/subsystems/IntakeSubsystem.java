// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//importing stuff for encoders
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue; //figure out why its red and fix it
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


//Dutcycle - how fast motor spins
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//to use constants file
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  //create intake motor
  private final TalonFX rollerMotor; //creates motor on roller

  private final TalonFX pivotIntakeMotor; //creates motor on pivot

  private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0); //regulates motor speed for roller, skeptical about pivot

  private final CANcoder pivotCANcoder; //pivot encoder

  private final CANBus canBus; //Creates canbus

  private final PositionVoltage pivotPosReq; //Creates Position Voltage request

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    rollerMotor = new TalonFX(Constants.Intake.motorId);

    canBus = new CANBus("rio");

    pivotIntakeMotor = new TalonFX(Constants.Intake.pivotId, canBus); 

    pivotCANcoder = new CANcoder(Constants.Intake.CANcoderId, canBus);

    pivotPosReq = new PositionVoltage(0);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Intake.kP;
    slot0Configs.kI = Constants.Intake.kI; 
    slot0Configs.kD = Constants.Intake.kD; 

    var MOCPivot = new MotorOutputConfigs();
    MOCPivot.Inverted = InvertedValue.CounterClockwise_Positive;
    MOCPivot.NeutralMode = NeutralModeValue.Brake;

    var feedbackConfigs = new FeedbackConfigs(); 
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; 
    feedbackConfigs.SensorToMechanismRatio = Constants.Intake.PivotSensorToMechanism; 

    var currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 30; //random number fix it

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0 = slot0Configs;
    pivotConfig.MotorOutput = MOCPivot;
    pivotConfig.Feedback = feedbackConfigs;
    pivotConfig.CurrentLimits = currentLimitsConfigs;
    pivotIntakeMotor.getConfigurator().apply(pivotConfig);

    CANcoderConfiguration pivotCANcoderConfig = new CANcoderConfiguration(); //Creates encoder configuration
    pivotCANcoderConfig.MagnetSensor = new MagnetSensorConfigs()
      .withMagnetOffset(0.0);
    pivotCANcoder.getConfigurator().apply(pivotCANcoderConfig);
    //Absolute encoder position --> internal encoder for pivot
    pivotIntakeMotor.setPosition(pivotCANcoder.getAbsolutePosition().getValueAsDouble());
 
  }

  public void setPivotPosition(double position) { //should not be red fix it
      pivotIntakeMotor.setControl(pivotPosReq.withPosition(position));
    }

    public void setPivotPosition(Angle position) {
      //uses angle measurement
      pivotIntakeMotor.setControl(pivotPosReq.withPosition(position));
    }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void start() {
    this.setPower(Constants.Intake.StartPower); //change this number placeholder 0
  }

  public void stop() {
    this.setPower(Constants.Intake.StopPower); //change this number placeholder 0
  }

  public void setPower(double power) {
    //write comments about rotations 1 rotation: 360 degrees
    rollerMotor.setControl(dutyCycleReq.withOutput(power));
  }

  
}
