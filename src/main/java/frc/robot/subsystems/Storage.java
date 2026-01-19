// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A subsystem for the storage. This has commands to run the rollers in the
 * storage forward, reverse, and to turn them off to manage the stored fuel.
 * 
 */
public class Storage extends SubsystemBase {
    private RollerState rollerState = RollerState.OFF;

    public Storage() {
        configureMotors();

    }

    private void configureMotors() {
        rollerMotor = new TalonFX(Constants.Storage.rollerMotorId);

        var motorOutput = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(35)
                        .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(motorOutput);

        rollerMotor.getConfigurator().apply(motorConfig);
    }

    public enum RollerState {
        FORWARD, //Forward to feed Shooter
        REVERSE, //Reverse to pass fuel
        OFF // rollers turned off
    }

    private TalonFX rollerMotor;

    public Command rollersForward() {
        return runOnce(
                () -> {
                    this.rollerState = RollerState.FORWARD;
                });
    }

    public Command rollersOff() {
        return runOnce(
                () -> {
                    this.rollerState = RollerState.OFF;
                });
    }

    public Command rollersReverse() {
        return runOnce(
                () -> {
                    this.rollerState = RollerState.REVERSE;
                });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        DutyCycleOut rollerOutput = new DutyCycleOut(0);
        switch (rollerState) {
            case FORWARD -> rollerOutput.Output = Constants.Storage.ROLLER_FORWARD_SPEED;
            case REVERSE -> rollerOutput.Output = Constants.Storage.ROLLER_REVERSE_SPEED;
            case OFF -> rollerOutput.Output = 0;
        }
        rollerMotor.setControl(rollerOutput);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
