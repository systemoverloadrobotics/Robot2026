package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Shooter.*;

public class ShooterSubSystem extends SubsystemBase {
    /**
     * ShooterSubsystem that controls:
     * - Flywheel velocity (RPM)
     * - Hood/Launch angle (degrees)
     * - Hood wheel output (percent)
     * Tracks targets and reports when each mechanism has reached its goal.
     */
    // ========== HARDWARE ==========
    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;
    private TalonFX hoodAngleMotor;
    private CANcoder hoodCANcoder;

    private VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
    private PositionVoltage hoodAngleRequest = new PositionVoltage(0);

    // ========== SETTERS ==========
    private double targetFlywheelFPS = 0.0;
    private double targetHoodAngleDegrees = 0.0;

    public ShooterSubSystem() {
        configureSubSystem();
    }

    // ========== MOTOR CONFIGURATION ==========
    private void configureSubSystem() {
        // Initialize motors
        flywheelMotor1 = new TalonFX(FLYWHEEL_MOTOR_ID);
        flywheelMotor2 = new TalonFX(FLYWHEEL_SECOND_MOTOR_ID);
        hoodAngleMotor = new TalonFX(HOOD_ANGLE_MOTOR_ID);

        //configure flywheel motor
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP =FLYWHEEL_kP;
        flywheelConfig.Slot0.kI = FLYWHEEL_kI;
        flywheelConfig.Slot0.kD = FLYWHEEL_kD;
        flywheelConfig.Slot0.kV = FLYWHEEL_kV;
        flywheelMotor1.getConfigurator().apply(flywheelConfig);
        flywheelMotor2.getConfigurator().apply(flywheelConfig);

        //configure hoodAnglemotor
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = HOOD_ANGLE_KP;
        hoodConfig.Slot0.kI = HOOD_ANGLE_KI;  
        hoodConfig.Slot0.kD = HOOD_ANGLE_KD; 
        // Neutral mode - brake to hold position
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Configured with FusedCANcoder feedback
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        hoodConfig.Feedback.FeedbackRemoteSensorID = SHOOTER_CanCoder;
        hoodConfig.Feedback.RotorToSensorRatio = HOOD_ANGLE_GEAR_RATIO;

        hoodAngleMotor.getConfigurator().apply(hoodConfig);

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
        hoodCANcoder.getConfigurator().apply(cancoderConfig);
    }

    public void setFlywheelVelocity(double feetPerSecond) {
        this.targetFlywheelFPS = feetPerSecond;
        // ft/s to RPS: ft/s ÷ circumference_ft × gear_ratio
        double rps = (feetPerSecond / FLYWHEEL_CIRCUMFERENCE_FT) * FLYWHEEL_GEAR_RATIO;
        flywheelMotor1.setControl(flywheelVelocityRequest.withVelocity(rps));
        flywheelMotor2.setControl(flywheelVelocityRequest.withVelocity(rps));
    }

    public void setHoodAngle(double degrees) {
        this.targetHoodAngleDegrees = degrees;
        // Convert degrees to motor rotations
        double rotations = degrees * HOOD_ANGLE_GEAR_RATIO / 360.0;
        hoodAngleMotor.setControl(hoodAngleRequest.withPosition(rotations));
    }

    public double getFlywheelVelocity() {
        double rps = flywheelMotor1.getVelocity().getValueAsDouble();
        return (rps / FLYWHEEL_GEAR_RATIO) * FLYWHEEL_CIRCUMFERENCE_FT;
    }

    public double getHoodAngle() {
        double rotations = hoodAngleMotor.getPosition().getValueAsDouble();
        return (rotations * 360.0) / HOOD_ANGLE_GEAR_RATIO;
    }

    public boolean isFlywheelAtTarget() {
        double error = Math.abs(getFlywheelVelocity() - targetFlywheelFPS);
        return error <= FLYWHEEL_FPS_TOLERANCE;
    }

    public boolean isHoodAngleAtTarget() {
        double error = Math.abs(getHoodAngle() - targetHoodAngleDegrees);
        return error <= HOOD_ANGLE_TOLERANCE;
    }
    public boolean isAtTarget() {
        return isFlywheelAtTarget() && isHoodAngleAtTarget();
    }
    public void stop() {
        targetFlywheelFPS = 0;
        flywheelMotor1.stopMotor();
        flywheelMotor2.stopMotor();
        hoodAngleMotor.stopMotor();
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
