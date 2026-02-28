package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
    private LinearVelocity targetFlywheelVelocity = FeetPerSecond.of(0.0);
    private AngularVelocity targetFlyWheelAngularVelocity = RotationsPerSecond.of(0.0);
    private Angle targetHoodAngleDegrees = Degrees.of(0.0);

    public ShooterSubSystem() {
        configureSubSystem();
    }

    // ========== MOTOR CONFIGURATION ==========
    private void configureSubSystem() {
        // Initialize motors
        flywheelMotor1 = new TalonFX(TOP_FLYWHEEL_ID);
        flywheelMotor2 = new TalonFX(BOTTOM_FLYWHEEL_ID);
        hoodAngleMotor = new TalonFX(SHOOTER_PIVOT_ID);
        hoodCANcoder = new CANcoder(SHOOTER_PIVOT_ENCODER);

        // configure flywheel motor
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP = FLYWHEEL_kP;
        flywheelConfig.Slot0.kI = FLYWHEEL_kI;
        flywheelConfig.Slot0.kD = FLYWHEEL_kD;
        flywheelConfig.Slot0.kV = FLYWHEEL_kV;
        flywheelConfig.Voltage.PeakForwardVoltage = 12.0;
        flywheelConfig.Voltage.PeakReverseVoltage = -12.0;
        flywheelConfig.Feedback.SensorToMechanismRatio = 1.0;
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast to allow flywheel to spin down naturally
        flywheelMotor1.getConfigurator().apply(flywheelConfig);
        flywheelMotor2.getConfigurator().apply(flywheelConfig);

        // configure hoodAnglemotor
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = HOOD_ANGLE_KP;
        hoodConfig.Slot0.kI = HOOD_ANGLE_KI;
        hoodConfig.Slot0.kD = HOOD_ANGLE_KD;
        // Neutral mode - brake to hold position
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // Configured with FusedCANcoder feedback
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        hoodConfig.Feedback.FeedbackRemoteSensorID = SHOOTER_PIVOT_ENCODER;
        hoodConfig.Feedback.SensorToMechanismRatio = SHOOTER_PIVOT_GEAR_RATIO;
        hoodConfig.Feedback.RotorToSensorRatio = 1.0;
        hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfig.Voltage.PeakForwardVoltage = 12.0;
        hoodConfig.Voltage.PeakReverseVoltage = -12.0;

        hoodAngleMotor.getConfigurator().apply(hoodConfig);

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cancoderConfig.MagnetSensor.MagnetOffset = 0.166;
        hoodCANcoder.getConfigurator().apply(cancoderConfig);
        hoodAngleMotor.setPosition(hoodCANcoder.getAbsolutePosition().getValue());
    }

    public void setFlywheelVelocity(LinearVelocity velocity) {
        this.targetFlywheelVelocity = velocity;
        // ft/s to RPS: (ft/s / circumference_ft) × gear_ratio
        this.targetFlyWheelAngularVelocity = RotationsPerSecond
                .of((velocity.in(FeetPerSecond) / FLYWHEEL_CIRCUMFERENCE_FT) * FLYWHEEL_GEAR_RATIO);
        flywheelMotor1.setControl(flywheelVelocityRequest.withVelocity(targetFlyWheelAngularVelocity));
        flywheelMotor2.setControl(flywheelVelocityRequest.withVelocity(targetFlyWheelAngularVelocity));
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        this.targetFlyWheelAngularVelocity = velocity;
        this.targetFlywheelVelocity = FeetPerSecond
                .of((velocity.in(RotationsPerSecond) * FLYWHEEL_CIRCUMFERENCE_FT) / FLYWHEEL_GEAR_RATIO);
        // ft/s to RPS: (ft/s / circumference_ft) × gear_ratio
        flywheelMotor1.setControl(flywheelVelocityRequest.withVelocity(targetFlyWheelAngularVelocity));
        flywheelMotor2.setControl(flywheelVelocityRequest.withVelocity(targetFlyWheelAngularVelocity));
    }

    public void setHoodAngle(Angle angle) {
        this.targetHoodAngleDegrees = angle;
        // Convert degrees to motor rotations
        double rotations = angle.in(Degrees) / 360.0;
        System.out.println(rotations);
        hoodAngleMotor.setControl(hoodAngleRequest.withPosition(rotations).withSlot(0));
    }

    public LinearVelocity getFlywheelVelocity() {
        var rps = flywheelMotor1.getVelocity().getValue();
        // return (rps / FLYWHEEL_GEAR_RATIO) * FLYWHEEL_CIRCUMFERENCE_FT;
        return FeetPerSecond.of(rps.in(RotationsPerSecond) / FLYWHEEL_GEAR_RATIO * FLYWHEEL_CIRCUMFERENCE_FT);
    }

    public Angle getHoodAngle() {
        return hoodAngleMotor.getPosition().getValue();
    }

    public boolean isFlywheelAtTarget() {
        double error = Math.abs(getFlywheelVelocity().in(FeetPerSecond) - targetFlywheelVelocity.in(FeetPerSecond));
        return error <= FLYWHEEL_FPS_TOLERANCE;
    }

    public boolean isHoodAngleAtTarget() {
        double error = Math.abs(getHoodAngle().in(Degrees) - targetHoodAngleDegrees.in(Degrees));
        return error <= SHOOTER_PIVOT_TOLERANCE;
    }

    public boolean isAtTarget() {
        return isFlywheelAtTarget() && isHoodAngleAtTarget();
    }

    @Override
    public void periodic() {
        DogLog.log("Shooter/FlywheelVelocity", getFlywheelVelocity().in(FeetPerSecond), FeetPerSecond);
        DogLog.log("Shooter/TargetFlywheelVelocity", targetFlywheelVelocity.in(FeetPerSecond), FeetPerSecond);
        DogLog.log("Shooter/TargetFlywheelAngularVelocity", targetFlyWheelAngularVelocity.in(RotationsPerSecond),
                RotationsPerSecond);
        DogLog.log("Shooter/HoodAngle", getHoodAngle().in(Degrees), Degrees);
    }
}
