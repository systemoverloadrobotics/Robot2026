package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.Test;
import frc.robot.Constants.Intake;

class IntakeSimTest {
  @Test
  void testIntakePivot() {
    HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    IntakeSubsystem intake = new IntakeSubsystem();

    // Command the intake to deploy position
    intake.setPivotPosition(Intake.IntakePosition);

    for (int i = 0; i < 200; i++) {
        Unmanaged.feedEnable(100);
        intake.simulationPeriodic();
        intake.periodic();
        Timer.delay(0.02);
    }

    double currentPos = intake.getIntakeCANCoderPosition().in(Degrees);
    double targetPos = Intake.IntakePosition.in(Degrees);

    System.out.println("Target: " + targetPos + ", Current: " + currentPos);
    // Verify that the pivot reached the desired position within the error bound
    assertTrue(Math.abs(currentPos - targetPos) < Intake.IntakeError.in(Degrees),
        "Intake pivot did not reach target. Target: " + targetPos + ", Current: " + currentPos);

    // Verify the atIntake() method works
    assertTrue(intake.atIntake(), "atIntake() should return true when at position");
  }
}
