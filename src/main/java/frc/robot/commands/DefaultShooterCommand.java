package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;
import frc.robot.RobotContainer.Mode;
import frc.robot.RobotContainer.Side;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.RollerState;
import frc.robot.subsystems.ShooterSubSystem;
import frc.robot.utils.ShooterCalculator;

public class DefaultShooterCommand extends Command {

    private final ShooterSubSystem shooter;
    private final Hopper hopper;
    private final Supplier<Distance> distanceSupplier;
    private final Supplier<Side> sideSupplier;
    private final Supplier<Mode> modeSupplier;
    private final BooleanSupplier isShootingSupplier;
    private final Supplier<Double> calibrationVelocitySupplier;
    private final Supplier<Double> calibrationAngleSupplier;

    private double lastCommandedHoodAngle = Double.NaN;
    private double lastCommandedFlywheelFps = Double.NaN;
    private RollerState lastRollerState = null;
    private boolean wasShootingLastCycle = false;

    private static final double HOOD_EPSILON = 0.1;
    private static final double FLYWHEEL_EPSILON = 0.5;

    public DefaultShooterCommand(
            ShooterSubSystem shooter,
            Hopper hopper,
            Supplier<Distance> distanceSupplier,
            Supplier<Side> sideSupplier,
            Supplier<Mode> modeSupplier,
            BooleanSupplier isShootingSupplier,
            Supplier<Double> calibrationVelocitySupplier,
            Supplier<Double> calibrationAngleSupplier) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.distanceSupplier = distanceSupplier;
        this.sideSupplier = sideSupplier;
        this.modeSupplier = modeSupplier;
        this.isShootingSupplier = isShootingSupplier;
        this.calibrationVelocitySupplier = calibrationVelocitySupplier;
        this.calibrationAngleSupplier = calibrationAngleSupplier;

        addRequirements(shooter, hopper);
    }

    @Override
    public void initialize() {
        lastCommandedHoodAngle = Double.NaN;
        lastCommandedFlywheelFps = Double.NaN;
        lastRollerState = null;
        wasShootingLastCycle = false;
    }

    @Override
    public void execute() {
        Mode mode = modeSupplier.get();
        Distance distance = distanceSupplier.get();
        Side side = sideSupplier.get();
        boolean isShooting = isShootingSupplier.getAsBoolean();

        SmartDashboard.putNumber("Target Shooting Distance (Ft)", distance.in(Units.Feet));
        SmartDashboard.putString("Mode", mode.toString());
        SmartDashboard.putString("Side", side.toString());

        LinearVelocity flywheelSpeed;
        Angle launchAngle;

        switch (mode) {
            case AUTO:
            case MANUAL:
                flywheelSpeed = ShooterCalculator.getRegressionVelocity(distance);
                launchAngle = ShooterCalculator.getRegressionAngle(distance);
                break;
            case CALIBRATION:
                flywheelSpeed = FeetPerSecond.of(calibrationVelocitySupplier.get());
                launchAngle = Degrees.of(calibrationAngleSupplier.get());
                break;
            default:
                return;
        }

        double adjustedAngleDeg = launchAngle.in(Degrees) * side.getDirection();
        Angle launchAngleAdjusted = Degrees.of(adjustedAngleDeg);

        // Lazy hood update — only send CAN frame when target changes
        if (Double.isNaN(lastCommandedHoodAngle)
                || Math.abs(adjustedAngleDeg - lastCommandedHoodAngle) > HOOD_EPSILON) {
            shooter.setHoodAngle(launchAngleAdjusted);
            lastCommandedHoodAngle = adjustedAngleDeg;
        }

        // Flywheel — only command when shooting, and only when target changes
        if (isShooting) {
            double newFps = flywheelSpeed.in(FeetPerSecond);
            if (adjustedAngleDeg < 0) {
                newFps = Math.abs(newFps);
            }
            if (Double.isNaN(lastCommandedFlywheelFps)
                    || Math.abs(newFps - lastCommandedFlywheelFps) > FLYWHEEL_EPSILON) {
                shooter.setFlywheelVelocity(FeetPerSecond.of(newFps));
                lastCommandedFlywheelFps = newFps;
            }
            wasShootingLastCycle = true;
        } else if (wasShootingLastCycle) {
            shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
            lastCommandedFlywheelFps = 0.0;
            wasShootingLastCycle = false;
        }

        // Hopper — only update when state changes
        RollerState desiredState = (shooter.isAtTarget() && isShooting)
                ? RollerState.FORWARD
                : RollerState.OFF;

        if (desiredState != lastRollerState) {
            hopper.setRollers(desiredState);
            lastRollerState = desiredState;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setFlywheelVelocity(FeetPerSecond.of(0.0));
        hopper.setRollers(RollerState.OFF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
