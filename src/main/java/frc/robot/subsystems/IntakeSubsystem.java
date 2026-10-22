// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;


import java.util.List;
import frc.robot.dashboard.NamedDevice;
import static edu.wpi.first.units.Units.Amps;
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
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

public class IntakeSubsystem extends SubsystemBase {

  // create intake motor
  private final TalonFX rollerMotor; // creates motor on roller


  private final TalonFX pivotIntakeMotor; // creates motor on pivot

  private final DutyCycleOut dutyCycleReq = new DutyCycleOut(0); // regulates motor speed for roller, skeptical about
                                                                 // pivot

  private final CANcoder pivotCANcoder; // pivot encoder

  private final CANBus canBus; // Creates canbus

  private final PositionVoltage pivotPosReq; // Creates Position Voltage request

  private final double maxPivotCurrent = 80;

  private Timer timer = new Timer();

  private int periodicCount = 0;

  private boolean runPivot = true;

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
    currentLimitsConfigs.StatorCurrentLimit = 110;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0 = slot0Configs;
    pivotConfig.MotorOutput = MOCPivot;
    pivotConfig.Feedback = feedbackConfigs;
    pivotConfig.CurrentLimits = currentLimitsConfigs;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.38;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;
    pivotConfig.Voltage.PeakForwardVoltage = Constants.Intake.PivotMaxVolts;
    pivotConfig.Voltage.PeakReverseVoltage = -Constants.Intake.PivotMaxVolts;
    pivotIntakeMotor.getConfigurator().apply(pivotConfig);

    CANcoderConfiguration pivotCANcoderConfig = new CANcoderConfiguration(); // Creates encoder configuration
    pivotCANcoderConfig.MagnetSensor = new MagnetSensorConfigs()
        .withMagnetOffset(Constants.Intake.PivotOffset)
        // CANcoder turns 2.5x the arm, so its [-0.15, 0.85) window is arm -21.6 to 122.4 deg.
        // Stowed is about -8.5 deg and deployed about 111 deg; at 0.75 the deployed reading
        // wrapped to -33 deg and the boot seed put the motor 141 deg off.
        .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.85))
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
    if (runPivot == false) {
      return;
    }

    pivotIntakeMotor.setControl(pivotPosReq.withPosition(position));
  }

  public void setPivotPosition(Angle position) {
    System.out.println("runPivot: " + runPivot);
    if (runPivot == false) {
      return;
    }
    // uses angle measurement
    //pivotIntakeMotor.setPosition(getIntakeCANCoderPosition(), 0.0);
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


  public void intakePushUp() {
    this.setPower(Constants.Intake.PushUpPower); // change this number placeholder 0
  }


  public void stop() {
    this.setPower(Constants.Intake.StopPower); // change this number placeholder 0
  }

  /**
   * Stops the roller and puts the pivot in neutral (brake). Phoenix keeps re-sending the last
   * request, so without this a deploy sent before a disable resumes as soon as the robot is enabled.
   */
  public void stopAll() {
    stop();
    pivotIntakeMotor.setControl(new NeutralOut());
  }

  public void setPower(double power) {
    rollerMotor.setControl(dutyCycleReq.withOutput(-power * Constants.Intake.RollerPowerScale));
  }

  @Override
  public void periodic() {
    /*if (periodicCount <= 50) {
      periodicCount += 1;
    } else {
      pivotIntakeMotor.setPosition(getIntakeCANCoderPosition());
      periodicCount = 0;
    }*/
    /*if((int)timer.get() == timer.get()){
      pivotIntakeMotor.setPosition(getIntakeCANCoderPosition());
    }*/

    /*if (pivotIntakeMotor.getStatorCurrent().getValue().in(Amps) > maxPivotCurrent) {
      pivotIntakeMotor.setControl(dutyCycleReq.withOutput(0.0));
      runPivot = false;
    }*/

    DogLog.log("Intake/AtIntake", atIntake());
    DogLog.log("Intake/RunPivot", runPivot);
    DogLog.log("Intake/IntakePosition", getIntakeCANCoderPosition());

    // DEBUG (intake not deploying): remove once fixed. Pivot positions are in arm degrees.
    double cancoderAbsRot = pivotCANcoder.getAbsolutePosition().getValueAsDouble();
    double pivotDeg = pivotIntakeMotor.getPosition().getValueAsDouble() * 360.0;
    double pivotTargetDeg = pivotIntakeMotor.getClosedLoopReference().getValueAsDouble() * 360.0;
    double pivotAmps = pivotIntakeMotor.getStatorCurrent().getValueAsDouble();
    double rollerOutput = rollerMotor.getDutyCycle().getValueAsDouble();
    DogLog.log("Intake/CANcoderAbsRot", cancoderAbsRot);
    DogLog.log("Intake/PivotMotorDeg", pivotDeg);
    DogLog.log("Intake/PivotTargetDeg", pivotTargetDeg);
    DogLog.log("Intake/PivotStatorAmps", pivotAmps);
    DogLog.log("Intake/RollerDutyCycle", rollerOutput);
    SmartDashboard.putBoolean("Intake/AtIntake", atIntake());
    SmartDashboard.putNumber("Intake/CANcoderAbsRot", cancoderAbsRot);
    SmartDashboard.putNumber("Intake/CANcoderArmDeg", getIntakeCANCoderPosition().in(Degrees));
    SmartDashboard.putNumber("Intake/PivotMotorDeg", pivotDeg);
    SmartDashboard.putNumber("Intake/PivotTargetDeg", pivotTargetDeg);
    SmartDashboard.putNumber("Intake/PivotStatorAmps", pivotAmps);
    SmartDashboard.putNumber("Intake/RollerDutyCycle", rollerOutput);

  }


  /** CAN devices for the dashboard's current and connection checks. */
  public List<NamedDevice> getDevices() {
    return List.of(
        NamedDevice.of("Intake roller", rollerMotor),
        NamedDevice.of("Intake pivot", pivotIntakeMotor),
        NamedDevice.of("Intake CANcoder", pivotCANcoder));
  }

  private SingleJointedArmSim pivotSim = new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          Constants.Intake.PivotSensorToMechanism,
          0.02, // MOI
          0.3, // length (m)
          Units.degreesToRadians(-20),
          Units.degreesToRadians(120),
          true,
          Units.degreesToRadians(0)); // starting angle

  private DCMotorSim rollerSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0), DCMotor.getKrakenX60(1));

  @Override
  public void simulationPeriodic() {
      var pivotSimState = pivotIntakeMotor.getSimState();
      var cancoderSimState = pivotCANcoder.getSimState();

      pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      cancoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      pivotSim.setInputVoltage(pivotSimState.getMotorVoltage());
      pivotSim.update(0.020);

      double posRads = pivotSim.getAngleRads();
      double posRotations = Units.radiansToRotations(posRads);

      // CANcoder turns 2.5x the arm
      cancoderSimState.setRawPosition(posRotations * 2.5);
      cancoderSimState.setVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * 2.5);

      pivotSimState.setRawRotorPosition(posRotations * Constants.Intake.PivotSensorToMechanism);
      pivotSimState.setRotorVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec()) * Constants.Intake.PivotSensorToMechanism);

      var rollerSimState = rollerMotor.getSimState();
      rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());
      rollerSim.update(0.020);
      rollerSimState.setRotorVelocity(Units.radiansToRotations(rollerSim.getAngularVelocityRadPerSec()));
  }
}