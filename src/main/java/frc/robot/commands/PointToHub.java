package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PointToHub extends Command {

    public static enum Alignment {
        LEFT(0),
        RIGHT(180);

        public final double angle;

        Alignment(double angle) {
            this.angle = angle;
        }
    }

    public static enum Strategy {
        SINGLE_TAG, // Use only one tag to align
        MULTI_TAG, // Use multiple tags to align
        FIELD // Use multiple tags and odometry to align
    }

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                             // second max angular
                                                                                             // velocity

    PhotonCamera leftCamera = new PhotonCamera(Constants.Vision.LEFT_CAMERA);
    PhotonCamera rightCamera = new PhotonCamera(Constants.Vision.RIGHT_CAMERA);

    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    PhotonPoseEstimator leftRobotPoseEstimator = new PhotonPoseEstimator(
            layout,
            Constants.Vision.ROBOT_TO_LEFT_CAMERA);

    PhotonPoseEstimator rightRobotPoseEstimator = new PhotonPoseEstimator(
            layout,
            Constants.Vision.ROBOT_TO_RIGHT_CAMERA);

    PhotonPoseEstimator leftShooterPoseEstimator = new PhotonPoseEstimator(
            layout,
            Constants.Vision.SHOOTER_TO_LEFT_CAMERA);

    PhotonPoseEstimator rightShooterPoseEstimator = new PhotonPoseEstimator(
            layout,
            Constants.Vision.SHOOTER_TO_RIGHT_CAMERA);

    ProfiledPIDController yawController;

    CommandSwerveDrivetrain drivetrain;
    CommandXboxController controller;

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    Transform3d tagToHub;

    private Field2d odometryField = new Field2d();

    // Align to this tag (for single-tag strategy)
    private int tag;

    private double yawOutput;

    private Distance xDistance = Feet.of(0.0);
    private Distance yDistance = Feet.of(0.0);

    private Alignment alignment;
    private Strategy strategy;

    private List<PhotonPipelineResult> leftResults;
    private List<PhotonPipelineResult> rightResults;

    private int controlsInverted = 1;

    private Distance previousDistance = Feet.of(0.0);
    private double previousDistanceTime = 0.0;
    private Timer distanceTimer = new Timer();

    private LinearVelocity shooterToHubVelocity = FeetPerSecond.of(0.0);

    public PointToHub(CommandSwerveDrivetrain drivetrain, CommandXboxController controller, int controlsInverted,
            Alignment alignment,
            Strategy strategy) {

        this.controller = controller;

        this.drivetrain = drivetrain;

        this.controlsInverted = controlsInverted;

        this.alignment = alignment;
        this.strategy = strategy;

        if (distanceTimer.isRunning() == false) {
            distanceTimer.start();
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            end(false);
            return;
        }

        if (alliance.get() == Alliance.Blue) {
            tag = Constants.Vision.BLUE_ALLIANCE_HUB_TAG;
            var t = Constants.Vision.BLUE_ALLIANCE_TAG_TO_HUB;
            this.tagToHub = new Transform3d(
                    new Translation3d(
                            t.getMeasureX(),
                            t.getMeasureY(),
                            Feet.of(0.0)),
                    new Rotation3d(
                            0.0,
                            0.0,
                            t.getRotation().getRadians()));
        } else {
            tag = Constants.Vision.RED_ALLIANCE_HUB_TAG;
            var t = Constants.Vision.RED_ALLIANCE_TAG_TO_HUB;
            this.tagToHub = new Transform3d(
                    new Translation3d(
                            t.getMeasureX(),
                            t.getMeasureY(),
                            Feet.of(0.0)),
                    new Rotation3d(
                            0.0,
                            0.0,
                            t.getRotation().getRadians()));
        }

        TrapezoidProfile.Constraints yawConstraints = new TrapezoidProfile.Constraints(1.33 * 0.75, 1.5 * 0.75);
        yawController = new ProfiledPIDController(7.0, 0, 0, yawConstraints);

        yawController.setTolerance(0.05 * 3);

        this.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        var driveState = drivetrain.getState();
        var drivePose = driveState.Pose;
        var driveSpeeds = driveState.Speeds;

        yawController.reset(drivePose.getRotation().getRadians(), driveSpeeds.omegaRadiansPerSecond);
    }

    public void updateResults() {
        this.leftResults = leftCamera.getAllUnreadResults();
        this.rightResults = rightCamera.getAllUnreadResults();
    }

    public void changeStrategy(Strategy strategy) {
        this.strategy = strategy;
    }

    public void singleTagStrategy() {

        if (leftResults.isEmpty() && rightResults.isEmpty()) {
            return;
        }

        Transform3d shooterToHub = null;

        if (!leftResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = leftResults.get(leftResults.size() - 1);

            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {

                    if (target.getFiducialId() != tag || target.getPoseAmbiguity() > 0.2) {
                        continue;
                    }

                    Transform3d cameraToTag = target.getBestCameraToTarget();
                    Transform3d shooterToCamera = Constants.Vision.SHOOTER_TO_LEFT_CAMERA;

                    shooterToHub = shooterToCamera.plus(cameraToTag).plus(tagToHub);
                    break;
                }
            }
        }

        if (shooterToHub == null && rightResults.isEmpty() == false) {
            var result = rightResults.get(rightResults.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == tag && target.getPoseAmbiguity() <= 0.2) {
                        Transform3d cameraToTag = target.getBestCameraToTarget();
                        Transform3d shooterToCamera = Constants.Vision.SHOOTER_TO_RIGHT_CAMERA;

                        shooterToHub = shooterToCamera.plus(cameraToTag).plus(tagToHub);
                        break;
                    }
                }
            }
        }

        if (shooterToHub == null) {
            control(this.yawOutput);
            return;
        }

        this.xDistance = shooterToHub.getTranslation().getMeasureX();
        this.yDistance = shooterToHub.getTranslation().getMeasureY();
        double targetYaw = Math.atan2(yDistance.in(Feet), xDistance.in(Feet));

        double currentYaw = drivetrain.getState().Pose.getRotation().getRadians() - Math.toRadians(alignment.angle);

        yawController.setGoal(targetYaw);
        this.yawOutput = yawController.calculate(currentYaw);

        control(this.yawOutput);
    }

    public void multiTagStrategy() {

        if (leftResults.isEmpty() && rightResults.isEmpty()) {
            return;
        }

        Transform3d shooterToHub = null;

        if (alignment.equals(Alignment.LEFT) && !leftResults.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = leftResults.get(leftResults.size() - 1);

            if (result.hasTargets()) {
                var visionEst = leftShooterPoseEstimator.estimateCoprocMultiTagPose(result);

                if (visionEst.isEmpty()) {
                    visionEst = leftShooterPoseEstimator.estimateLowestAmbiguityPose(result);
                }

                if (visionEst.isPresent()) {
                    var fieldToShooter = visionEst.get().estimatedPose;
                    var fieldToTag = layout.getTagPose(tag);

                    var shooterToTag = new Transform3d(fieldToShooter, fieldToTag.get());
                    shooterToHub = shooterToTag.plus(tagToHub);
                }
            }
        }

        if (alignment.equals(Alignment.RIGHT) && !rightResults.isEmpty()) {
            var result = rightResults.get(rightResults.size() - 1);
            if (result.hasTargets()) {
                var visionEst = rightShooterPoseEstimator.estimateCoprocMultiTagPose(result);

                if (visionEst.isEmpty()) {
                    visionEst = rightShooterPoseEstimator.estimateLowestAmbiguityPose(result);
                }

                if (visionEst.isPresent()) {
                    var fieldToShooter = visionEst.get().estimatedPose;
                    var fieldToTag = layout.getTagPose(tag);

                    var shooterToTag = new Transform3d(fieldToShooter, fieldToTag.get());
                    shooterToHub = shooterToTag.plus(tagToHub);
                }
            }
        }

        if (shooterToHub == null) {
            control(this.yawOutput);
            return;
        }

        this.xDistance = shooterToHub.getTranslation().getMeasureX();
        this.yDistance = shooterToHub.getTranslation().getMeasureY();
        double targetYaw = Math.atan2(yDistance.in(Feet), xDistance.in(Feet));

        double currentYaw = drivetrain.getState().Pose.getRotation().getRadians() - Math.toRadians(alignment.angle);

        yawController.setGoal(targetYaw);
        this.yawOutput = yawController.calculate(currentYaw);

        control(this.yawOutput);
    }

    public void fieldStrategy() {

        var fieldToRobot = new Pose3d(drivetrain.getState().Pose);
        var fieldToShooter = fieldToRobot.plus(Vision.ROBOT_TO_LEFT_SHOOTER);
        if (alignment.equals(Alignment.RIGHT)) {
            fieldToShooter = fieldToRobot.plus(Vision.ROBOT_TO_RIGHT_SHOOTER);
        }
        var fieldToHub = layout.getTagPose(tag).get().plus(tagToHub);

        var shooterToHub = new Transform3d(fieldToShooter, fieldToHub);

        this.xDistance = shooterToHub.getTranslation().getMeasureX();
        this.yDistance = shooterToHub.getTranslation().getMeasureY();
        double targetYaw = Math.atan2(yDistance.in(Feet), xDistance.in(Feet));

        double currentYaw = drivetrain.getState().Pose.getRotation().getRadians() - Math.toRadians(alignment.angle);

        yawController.setGoal(targetYaw);
        this.yawOutput = yawController.calculate(currentYaw);

        control(this.yawOutput);
    }

    @Override
    public void execute() {
        // update();

        switch (strategy) {
            case SINGLE_TAG:
                singleTagStrategy();
                break;
            case MULTI_TAG:
                multiTagStrategy();
                break;
            case FIELD:
                fieldStrategy();
                break;
        }

        odometryField.setRobotPose(drivetrain.getState().Pose);
        SmartDashboard.putData("Swerve/OdometryPose", odometryField);

        logPose();
        updateShooterVelocityToHub();
    }

    private void updateShooterVelocityToHub() {
        Distance newDistance = getDistance();
        shooterToHubVelocity = FeetPerSecond
                .of((newDistance.in(Feet) - previousDistance.in(Feet)) / (distanceTimer.get() - previousDistanceTime));
        previousDistance = newDistance;
        previousDistanceTime = distanceTimer.get();
    }

    public void update() {
        updateResults();
        updatePose(this.leftResults, leftRobotPoseEstimator);
        updatePose(this.rightResults, rightRobotPoseEstimator);
    }

    public Distance getDistance() {
        double x = this.xDistance.in(Feet);
        double y = this.yDistance.in(Feet);
        return Feet.of(Math.hypot(x, y));
    }

    public void control(double yawOutput) {

        drive.withRotationalRate(yawOutput)
                .withVelocityX(controller.getLeftY() * MaxSpeed * controlsInverted)
                .withVelocityY(controller.getLeftX() * MaxSpeed * controlsInverted);

        drivetrain.setControl(
                drive);
    }

    public void updatePose(List<PhotonPipelineResult> results, PhotonPoseEstimator photonEstimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : results) {

            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            Pose2d visionPose = visionEst.get().estimatedPose.toPose2d();

            // TODO: Scale vision x,y stdev by distance
            drivetrain.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(visionEst.get().timestampSeconds),
                    Vision.DEFAULT_VISIONSTDEV.apply(result));
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (drivetrain != null) {
            drivetrain.setControl(new SwerveRequest.FieldCentric().withRotationalRate(0));
        }

        super.end(interrupted);
    }

    public void resetTranslationPoseWithVision() {
        updateResults();

        if (alignment.equals(Alignment.LEFT)) {
            resetTranslationPoseWithVision(this.leftResults, leftRobotPoseEstimator);
        } else {
            resetTranslationPoseWithVision(this.rightResults, rightRobotPoseEstimator);
        }
    }

    public void resetTranslationPoseWithVision(List<PhotonPipelineResult> results, PhotonPoseEstimator poseEstimator) {
        if (drivetrain == null) {
            return;
        }

        var speeds = drivetrain.getState().Speeds;
        if (speeds.omegaRadiansPerSecond > 0.5 || speeds.omegaRadiansPerSecond < -0.5 || speeds.vxMetersPerSecond > 1.0
                || speeds.vxMetersPerSecond < -1.0 || speeds.vyMetersPerSecond > 1.0
                || speeds.vyMetersPerSecond < -1.0) {
            System.out.println("ERROR: Robot is moving too fast to reset pose with vision.");
            return;
        }

        if (results.isEmpty()) {
            System.out.println("ERROR: No vision results to reset pose with.");
            return;
        }

        var result = results.get(results.size() - 1);

        if (result.hasTargets()) {
            var visionEst = poseEstimator.estimateCoprocMultiTagPose(result);

            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            if (visionEst.isPresent()) {
                var fieldToRobot = visionEst.get().estimatedPose;

                drivetrain.resetTranslation(new Translation2d(
                        fieldToRobot.getMeasureX(),
                        fieldToRobot.getMeasureY()));
            }
        }

    }

    public LinearVelocity getVelocity() {
        return shooterToHubVelocity;
    }

    public void logPose() {
        DogLog.log("PointToHub/YawOutput", this.yawOutput);
        DogLog.log("PointToHub/XDistance", this.xDistance.in(Feet), Feet);
        DogLog.log("PointToHub/YDistance", this.yDistance.in(Feet), Feet);
        DogLog.log("PointToHub/Distance", getDistance().in(Feet), Feet);
        DogLog.log("PointToHub/CurrentYaw",
                Math.toDegrees(drivetrain.getState().Pose.getRotation().getRadians()), Degrees);
        DogLog.log("PointToHub/TargetYaw",
                Math.toDegrees(yawController.getGoal().position), Degrees);
        // DogLog.log("PointToHub/AlignmentSide", alignment.toString());
        DogLog.log("PointToHub/AlignmentSide", alignment);
        DogLog.log("PointToHub/Strategy", strategy);
        SmartDashboard.putString("Vision Strategy", strategy.toString());
    }
}