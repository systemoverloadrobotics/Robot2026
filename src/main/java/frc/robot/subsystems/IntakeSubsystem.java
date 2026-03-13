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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class IntakeSubsystem extends SubsystemBase {

  // create intake motor
  private final TalonFX rollerMotor; // creates motor on roller

  private final TalonFX pivotIntakeMotor; // creates motor on pivot

  private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0);      // roller

  private final CANcoder pivotCANcoder; // pivot encoder

  private final CANBus canBus; // Creates canbus

  private final PositionVoltage pivotPosReq; // Creates Position Voltage request

  private final double maxPivotVoltage = 12.0;

  private final double maxPivotCurrent = 90;

  private double lastRollerPower = 0;

  private Angle lastPivotPosition = null;

  private double simCANCoderPositionDeg = 0.0;

  private boolean simInitialized = false;

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

    var feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feedbackConfigs.SensorToMechanismRatio = Constants.Intake.PivotSensorToMechanism;

    var currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = 30;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.StatorCurrentLimit = maxPivotCurrent;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0 = slot0Configs;
    pivotConfig.MotorOutput = MOCPivot;
    pivotConfig.Feedback = feedbackConfigs;
    pivotConfig.CurrentLimits = currentLimitsConfigs;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.38;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;
    pivotConfig.Voltage.PeakForwardVoltage = maxPivotVoltage;
    pivotConfig.Voltage.PeakReverseVoltage = -maxPivotVoltage;
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

  /**
   * Reads the Intake cancorder position and applies the gear ratio as arm:cancorder = 1:2.5
   * @return
   */

  public Angle getIntakeCANCoderPosition() {
    if (RobotBase.isSimulation()) {
      return Degrees.of(simCANCoderPositionDeg);
    }
    return pivotCANcoder.getAbsolutePosition().getValue().div(2.5);
  }

  /**
   * Re-seeds the rotor encoder from the CANcoder absolute position.
   * Call this on robot enable so that arm movement while disabled doesn't cause drift.
   */
  public void reseedFromCANcoder() {
    pivotIntakeMotor.setPosition(getIntakeCANCoderPosition());
  }

  /**
   * Move the motor to requested PositionVoltage using PID
   * @param position
   */

  public void setPivotPosition(Angle position) {
    lastPivotPosition = position;
    pivotIntakeMotor.setControl(pivotPosReq.withPosition(position));
  }

  /**
   * Return true if the arm is in intake position defined as 106 degrees with a tolerance of IntakeError
   * @return
   */

  public boolean atIntake() {
    var goalPosition = Intake.IntakePosition.in(Degrees);
    // Use the rotor encoder (same source the PID runs on) to avoid stalling
    // when rotor and CANcoder drift apart after startup seeding.
    // var actualPosition = getIntakeCANCoderPosition().in(Degrees);
    var actualPosition = pivotIntakeMotor.getPosition().getValue().in(Degrees);
    var error = Intake.IntakeError.in(Degrees);
    return Math.abs(goalPosition - actualPosition) < error;
  }

  public void start() {
    this.setPower(Constants.Intake.StartPower);
  }

  public void stop() {
    this.setPower(Constants.Intake.StopPower);
  }

  public void setPower(double power) {
    lastRollerPower = power;
    rollerMotor.setControl(dutyCycleReq.withOutput(-power));
  }

  @Override
  public void simulationPeriodic() {
    // Seed from actual position on first call so we start from the right place
    if (!simInitialized) {
      simCANCoderPositionDeg = getIntakeCANCoderPosition().in(Degrees);
      simInitialized = true;
    }
    rollerMotor.getSimState().setSupplyVoltage(12.0);

    if (lastPivotPosition == null) return;

    var motorSim = pivotIntakeMotor.getSimState();
    motorSim.setSupplyVoltage(12.0);

    // Increment internal sim position toward target
    double targetMechDeg = lastPivotPosition.in(Degrees);
    double error = targetMechDeg - simCANCoderPositionDeg;
    double stepDeg = Math.copySign(Math.min(Math.abs(error), 0.5), error);
    simCANCoderPositionDeg += stepDeg;
  }

  @Override
  public void periodic() {
    DogLog.log("Intake/CommandedPivotPosition", lastPivotPosition != null ? lastPivotPosition.in(Degrees) : Double.NaN);
    DogLog.log("Intake/AtIntake", atIntake());
    DogLog.log("Intake/CANCoderPosition", getIntakeCANCoderPosition().in(Degrees));
    DogLog.log("Intake/RollerOutput", rollerMotor.getDutyCycle().getValue());
    DogLog.log("Intake/RollerCommandedPower/AtIntake", atIntake() ? lastRollerPower : Double.NaN);
    DogLog.log("Intake/RollerCommandedPower/NotAtIntake", atIntake() ? Double.NaN : lastRollerPower);
  }

}
