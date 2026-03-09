// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

//Dutcycle - how fast motor spins
import com.ctre.phoenix6.controls.DutyCycleOut;

public class IntakeSubsystem extends SubsystemBase {

  // create intake motor
  private final TalonFX rollerMotor; // creates motor on roller


  private final TalonFX pivotIntakeMotor; // creates motor on pivot

  private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0); // regulates motor speed for roller, skeptical about
                                                                 // pivot

  private final CANcoder pivotCANcoder; // pivot encoder

  private final CANBus canBus; // Creates canbus

  private final PositionVoltage pivotPosReq; // Creates Position Voltage request

  /** Creates a new Intake. */
  public IntakeSubsystem() {

    rollerMotor = new TalonFX(Constants.Intake.ROLLER_ID);

    canBus = new CANBus("rio");
    pivotIntakeMotor = new TalonFX(Constants.Intake.PIVOT_ID, canBus);
    pivotCANcoder = new CANcoder(Constants.Intake.ENCODER_ID, canBus);
    pivotPosReq = new PositionVoltage(0);

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = Constants.Intake.KP;
    slot0Configs.kI = Constants.Intake.KI;
    slot0Configs.kD = Constants.Intake.KD;

    var MOCPivot = new MotorOutputConfigs();
    MOCPivot.Inverted = InvertedValue.CounterClockwise_Positive;
    MOCPivot.NeutralMode = NeutralModeValue.Brake;

    var rollerMotorConfig = new TalonFXConfiguration();
    rollerMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimit = 90;
    rollerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(rollerMotorConfig);

    // initialize spindexer motor


    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.SensorToMechanismRatio = Constants.Intake.PivotSensorToMechanism;

    var currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 30;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0 = slot0Configs;
    pivotConfig.MotorOutput = MOCPivot;
    pivotConfig.Feedback = feedbackConfigs;
    pivotConfig.CurrentLimits = currentLimitsConfigs;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.38;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;
    pivotIntakeMotor.getConfigurator().apply(pivotConfig);

    CANcoderConfiguration pivotCANcoderConfig = new CANcoderConfiguration(); // Creates encoder configuration
    pivotCANcoderConfig.MagnetSensor = new MagnetSensorConfigs()
        .withMagnetOffset(Constants.Intake.PivotOffset)
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.75))
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    pivotCANcoder.getConfigurator().apply(pivotCANcoderConfig);
    // Absolute encoder position --> internal encoder for pivot
    pivotIntakeMotor.setPosition(getIntakeCANCoderPosition());

  }

  public Angle getIntakeCANCoderPosition() {
    //return pivotCANcoder.getPosition().getValue().plus(Rotations.of(1.635));
    //return pivotIntakeMotor.getPosition().getValue();
    return pivotCANcoder.getAbsolutePosition().getValue().div(2.5);
  }

  public void setPivotPosition(double position) { // should not be red fix it
    pivotIntakeMotor.setControl(pivotPosReq.withPosition(position));
  }

  public void setPivotPosition(Angle position) {
    // uses angle measurement
    pivotIntakeMotor.setPosition(getIntakeCANCoderPosition());
    pivotIntakeMotor.setControl(pivotPosReq.withPosition(position));
  }

  public boolean atIntake() {
    var goalPosition = Intake.IntakePosition.in(Degrees);
    var actualPosition = getIntakeCANCoderPosition().in(Degrees);
    var error = Intake.IntakeError.in(Degrees);

    return Math.abs(goalPosition - actualPosition) < error;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  public void start() {
    this.setPower(Constants.Intake.StartPower); // change this number placeholder 0
  }

  public void stop() {
    this.setPower(Constants.Intake.StopPower); // change this number placeholder 0
  }

  public void setPower(double power) {
    rollerMotor.setControl(dutyCycleReq.withOutput(power));
  }

  @Override
  public void periodic() {
    pivotIntakeMotor.setPosition(getIntakeCANCoderPosition());
  }

}
