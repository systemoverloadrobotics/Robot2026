// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A subsystem for the storage. This has commands to run the rollers in the
 * storage forward, reverse, and to turn them off to manage the stored fuel.
 * 
 */
public class Hopper extends SubsystemBase {
    private TalonFX rollerMotor;
    private DutyCycleOut rollerOutput = new DutyCycleOut(0);
    private TalonFX spindexerMotor; 
    private DutyCycleOut spindexerOutput = new DutyCycleOut(0);

    public Hopper() {
        configureMotors();
    }

    private void configureMotors() {
        rollerMotor = new TalonFX(Constants.Hopper.ROLLER_MOTOR_ID);
        spindexerMotor = new TalonFX(Constants.Hopper.SPINDEXER_ID);

        var spindexerMotorConfig = new TalonFXConfiguration();
        spindexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
        spindexerMotorConfig.CurrentLimits.StatorCurrentLimit = 90;
        spindexerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        spindexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        spindexerMotor.getConfigurator().apply(spindexerMotorConfig);

        var MOCRoller = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
        TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(MOCRoller);

        rollerMotor.getConfigurator().apply(motorConfig);
    }

    public enum RollerState {
        FORWARD(Constants.Hopper.ROLLER_FORWARD_SPEED), //Forward to feed Shooter
        REVERSE(Constants.Hopper.ROLLER_REVERSE_SPEED), //Reverse to pass fuel
        OFF(0); // rollers turned off
        
        private double rollerSpeed;
        
        private RollerState(double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }

        public String getDescription() {
            return "" + this.rollerSpeed;
        }
        
    }

    public void setRollers(RollerState rollerState) {
        rollerMotor.setControl(rollerOutput.withOutput(rollerState.rollerSpeed));
        spindexerMotor.setControl(spindexerOutput.withOutput(-rollerState.rollerSpeed));
    }
}
