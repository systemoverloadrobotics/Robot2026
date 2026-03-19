package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class AlignmentTestManager {

    // drive speed and durations - move 1 meter square 
    private static final double DRIVE_SPEED_MPS     = 0.50; // speed in meters per second
    private static final double RUN_DURATION_SEC   = 2.0;   // how long to run 
    private static final double PAUSE_DURATION_SEC   = 0.3; // pause duration 
    private static final double ROTATE_RATE_RPS     = 1.0;  // rotaion per second
    private static final double ROTATE_DURATION_SEC = (2 * Math.PI) / ROTATE_RATE_RPS + 0.1;

    // NT table creation 
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("AlignmentTest");

    // Robot (this class) reads input from NT using these variables
    private final StringSubscriber  testNameSub;     // test to run starightLine, square, rotate etc
    private final BooleanSubscriber runTriggerSub;   // true will fire 

    // Robot outputs 
    private final StringPublisher  testNamePub;      // show what test is running 
    private final BooleanPublisher runTriggerPub;    // set to false after firing
    private final StringPublisher  testStatusPub;    // one of the status - IDLE / RUNNING / COMPLETE

    // robot gyro publishes to NT
    private final DoublePublisher gyroYawPub;
    private final DoublePublisher gyroPitchPub;
    private final DoublePublisher gyroRollPub;
    private final DoublePublisher gyroYawDeltaPub;

    // swerve module publishes to NT - target, actual and error angle for each wheel 
    private static final String[] MOD_NAMES = {"FL", "FR", "BL", "BR"};
    private final DoublePublisher[] modAngleActual = new DoublePublisher[4];
    private final DoublePublisher[] modAngleTarget = new DoublePublisher[4];
    private final DoublePublisher[] modAngleError  = new DoublePublisher[4];
    private final DoublePublisher[] modSpeedActual = new DoublePublisher[4];
    private final DoublePublisher[] modSpeedTarget = new DoublePublisher[4];

    // Swerve request - field centric and use velocity instead of voltage 0.5
    // needed for calibration 
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
        
    // driveTrain.getPigeon2() - read gyro 
    // driveTrain.seedFieldCentric() - reset odometry 
    // driveTrain.getState().ModuleStates - get angles and velocity of each swerve module 
    // driveTrain.applyRequest(command) - sends a command 
    private final CommandSwerveDrivetrain driveTrain;

    private Command activeTest = null;

    private double  gyroYawAtTestStart = 0.0; // store gyro value at the start 

    
    public AlignmentTestManager(CommandSwerveDrivetrain drivetrain) {

        this.driveTrain = drivetrain;

        // set selectTest to one of - strafe | square | rotate | straightLine
        // set runTrigger to true to fire
        testNamePub  = table.getStringTopic("selectTest").publish();
        testNameSub  = table.getStringTopic("selectTest").subscribe("none");
        testNamePub.set("none");

        runTriggerPub = table.getBooleanTopic("runTrigger").publish();
        runTriggerSub = table.getBooleanTopic("runTrigger").subscribe(false);
        runTriggerPub.set(false);

        // outpyut writer for status 
        testStatusPub = table.getStringTopic("status").publish();
        testStatusPub.set("IDLE");

        // reader for gyro 
        gyroYawPub      = table.getDoubleTopic("gyro/yaw_deg").publish();
        gyroPitchPub    = table.getDoubleTopic("gyro/pitch_deg").publish();
        gyroRollPub     = table.getDoubleTopic("gyro/roll_deg").publish();
        gyroYawDeltaPub = table.getDoubleTopic("gyro/yawDeltaFromStart_deg").publish();

        // per module angle and speed in meter per sec writer 
        for (int i = 0; i < 4; i++) {
            String prefix = "modules/" + MOD_NAMES[i] + "/";
            modAngleActual[i] = table.getDoubleTopic(prefix + "angleActual_deg").publish();
            modAngleTarget[i] = table.getDoubleTopic(prefix + "angleTarget_deg").publish();
            modAngleError[i]  = table.getDoubleTopic(prefix + "angleError_deg").publish();
            modSpeedActual[i] = table.getDoubleTopic(prefix + "speedActual_mps").publish();
            modSpeedTarget[i] = table.getDoubleTopic(prefix + "speedTarget_mps").publish();
        }
    }

    public void periodic() {

        // check truigger 
        if (runTriggerSub.get() && activeTest == null) {
            String selected = testNameSub.get();

            if (!selected.equals("none")) {
                activeTest = selectTest(selected);

                if (activeTest != null) {
                    gyroYawAtTestStart = driveTrain.getPigeon2().getYaw().getValueAsDouble();
                    driveTrain.seedFieldCentric();
                    CommandScheduler.getInstance().schedule(activeTest);
                    testStatusPub.set("RUNNING: " + selected);
                }
            }

            // Always reset trigger so it doesn't re-fire
            runTriggerPub.set(false);
        }

        // is test running? if so, is it complete 
        if (activeTest != null && activeTest.isFinished()) {
            testStatusPub.set("COMPLETE");
            activeTest = null;
        }

        // log metrics 
        logGyro();    
        logModules(); 
    }

    private Command selectTest(String name) {
        return switch (name) {
            case "square"       -> driveSquare();
            case "strafe"       -> pureStrafe();
            case "rotate"       -> spinInPlace();
            case "straightLine" -> straightLine();
            default             -> null;
        };
    }

    private void logGyro() {

        if (Robot.isSimulation()) {
            // Use odometry rotation instead of gyro in sim
            double simYaw = driveTrain.getState().Pose.getRotation().getDegrees();
            gyroYawPub.set(simYaw);
            gyroPitchPub.set(0.0);
            gyroRollPub.set(0.0);
            gyroYawDeltaPub.set(activeTest != null ? MathUtil.inputModulus(simYaw - gyroYawAtTestStart, -180.0, 180.0) : 0.0);
            return;
        }        

        Pigeon2 pigeon = driveTrain.getPigeon2();
        if (pigeon == null) 
            return;

        double  currentYaw = pigeon.getYaw().getValueAsDouble();
        gyroYawPub.set(currentYaw);
        gyroPitchPub.set(pigeon.getPitch().getValueAsDouble());
        gyroRollPub.set(pigeon.getRoll().getValueAsDouble());

        // delta only meaningful while a test is active — zero when idle
        if (activeTest != null) {
            double delta = MathUtil.inputModulus(currentYaw - gyroYawAtTestStart, -180.0, 180.0);
            gyroYawDeltaPub.set(delta);
        } else {
            gyroYawDeltaPub.set(0.0);
        }
    }

    private void logModules() {
        SwerveModuleState[] actual  = driveTrain.getState().ModuleStates;
        SwerveModuleState[] targets = driveTrain.getState().ModuleTargets;

        if (actual == null || targets == null) return;
        if (actual.length < 4 || targets.length < 4) return;

        for (int i = 0; i < 4; i++) {
            double actualAngle = actual[i].angle.getDegrees();
            double targetAngle = targets[i].angle.getDegrees();

            double error = MathUtil.inputModulus(actualAngle - targetAngle, -180.0, 180.0);

            modAngleActual[i].set(actualAngle);
            modAngleTarget[i].set(targetAngle);
            modAngleError[i].set(error);
            modSpeedActual[i].set(actual[i].speedMetersPerSecond);
            modSpeedTarget[i].set(targets[i].speedMetersPerSecond);
        }
    }    

    // tests commands 

    private Command straightLine() {
        return Commands.sequence(
            drive( DRIVE_SPEED_MPS,0,0, RUN_DURATION_SEC),
            drive( 0,0, 0, PAUSE_DURATION_SEC),
            drive(-DRIVE_SPEED_MPS,0, 0, RUN_DURATION_SEC),
            drive(0, 0, 0, PAUSE_DURATION_SEC)
        );
    }

    private Command pureStrafe() {
        return Commands.sequence(
            drive(0, DRIVE_SPEED_MPS,0, RUN_DURATION_SEC),
            drive(0, 0,0, PAUSE_DURATION_SEC),
            drive( 0, -DRIVE_SPEED_MPS,0, RUN_DURATION_SEC),
            drive(0, 0, 0, PAUSE_DURATION_SEC)
        );
    }

    private Command spinInPlace() {
        return Commands.sequence(
            drive( 0, 0, ROTATE_RATE_RPS, ROTATE_DURATION_SEC),
            drive( 0, 0, 0, PAUSE_DURATION_SEC)
        );
    }

    private Command driveSquare() {
        return Commands.sequence(
            drive( DRIVE_SPEED_MPS, 0, 0, RUN_DURATION_SEC),
            drive( 0, 0, 0, PAUSE_DURATION_SEC),
            drive( 0, -DRIVE_SPEED_MPS, 0, RUN_DURATION_SEC),
            drive( 0, 0, 0, PAUSE_DURATION_SEC),
            drive(-DRIVE_SPEED_MPS,  0, 0, RUN_DURATION_SEC),
            drive( 0, 0, 0, PAUSE_DURATION_SEC),
            drive( 0, DRIVE_SPEED_MPS, 0, RUN_DURATION_SEC),
            drive( 0, 0, 0, PAUSE_DURATION_SEC)
        );
    }

    private Command drive(double vx, double vy, double rot, double seconds) {
        return driveTrain.applyRequest(() ->
            driveRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(rot)
        ).withTimeout(seconds);
    }
}