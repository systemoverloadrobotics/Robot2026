package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private TalonFX flywheelLeaderMotor;
    private TalonFX flywheelFollowerMotor;
    private TalonFX hoodAngleMotor;

    private VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
    private PositionVoltage hoodAngleRequest = new PositionVoltage(0);

    private VoltageOut flywheelOutput = new VoltageOut(0);
    private VoltageOut hoodAngleOutput = new VoltageOut(0);

    // ========== SETTERS ==========
    private double targetFlywheelRPM = 0.0;
    private double targetHoodAngleDegrees = 0.0;

    public ShooterSubSystem() {
        configureSubSystem();
    }

    // ========== MOTOR CONFIGURATION ==========
    private void configureSubSystem() {
        // Initialize motors
        flywheelLeaderMotor = new TalonFX(FLYWHEEL_MOTOR_ID);
        flywheelFollowerMotor = new TalonFX(FLYWHEEL_SECOND_MOTOR_ID);
        hoodAngleMotor = new TalonFX(HOOD_ANGLE_MOTOR_ID);

        //configure flywheel motor
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.Slot0.kP =FLYWHEEL_kP;
        flywheelConfig.Slot0.kI = FLYWHEEL_kI;
        flywheelConfig.Slot0.kD = FLYWHEEL_kD;
        flywheelConfig.Slot0.kV = FLYWHEEL_kV;
        flywheelLeaderMotor.getConfigurator().apply(flywheelConfig);
        flywheelFollowerMotor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

        //configure hoodAnglemotor
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = HOOD_ANGLE_KP;
        // Neutral mode - brake to hold position
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodAngleMotor.getConfigurator().apply(hoodConfig);
    }

    public void setFlywheelVelocity(double rpm) {
        this.targetFlywheelRPM = rpm;
        // Convert RPM to rotations per second for TalonFX
        double rps = (rpm / 60.0) * FLYWHEEL_GEAR_RATIO;
        flywheelLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(rps));
    }

    public void setHoodAngle(double degrees) {
        this.targetHoodAngleDegrees = degrees;
        // Convert degrees to motor rotations
        double rotations = degrees * HOOD_ANGLE_GEAR_RATIO / 360.0;
        hoodAngleMotor.setControl(hoodAngleRequest.withPosition(rotations));
    }

    public double getFlywheelVelocity() {
        double rps = flywheelLeaderMotor.getVelocity().getValueAsDouble();
        return (rps * 60.0) / FLYWHEEL_GEAR_RATIO;
    }

    public double getHoodAngle() {
        double rotations = hoodAngleMotor.getPosition().getValueAsDouble();
        return (rotations * 360.0) / HOOD_ANGLE_GEAR_RATIO;
    }

    public boolean isFlywheelAtTarget() {
        double error = Math.abs(getFlywheelVelocity() - targetFlywheelRPM);
        return error <= FLYWHEEL_RPM_TOLERANCE;
    }

    public boolean isHoodAngleAtTarget() {
        double error = Math.abs(getHoodAngle() - targetHoodAngleDegrees);
        return error <= HOOD_ANGLE_TOLERANCE;
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
