// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radian;

import java.util.function.Function;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
    public static class Intake {
      public static final double KP = 60.0;
      public static final double KI = 0;
      public static final double KD = 0;
      public static final int ROLLER_ID = 32; //change this number 0 is placeholder
      public static final int PIVOT_ID = 31; //need to change number
      public static final int ENCODER_ID = 33; //need to change number
      public static final int PivotPID = 0; //need to change number
      public static final double PivotSensorToMechanism = 26.66666667;//need to change number
      public static final double StartPower = 0.5;//need to change number
      public static final double StopPower = 0; //need to change number
      public static final double OuttakePower = -0.5; // Negative to reverse direction
    }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

   public static class Vision {
    public static final String LEFT_CAMERA = "LeftCamera";
    public static final String RIGHT_CAMERA = "RightCamera";

    // TODO: Update Transforms where bot starts on left side of shooter
    public static final Transform3d LEFT_CAMERA_TO_SHOOTER = new Transform3d(
        new Translation3d(
            Inches.of(-17.5),
            Inches.of(-3),
            Inches.of(0.45)),
        new Rotation3d(
            Degrees.of(0),
            Degrees.of(-30),
            Degrees.of(0)));
    public static final Transform3d RIGHT_CAMERA_TO_SHOOTER = new Transform3d(
        new Translation3d(
            Inches.of(-17.5),
            Inches.of(3),
            Inches.of(0.45)),
        new Rotation3d(
            Degrees.of(0),
            Degrees.of(-30),
            Degrees.of(0)));

    public static final Transform3d SHOOTER_TO_LEFT_CAMERA = LEFT_CAMERA_TO_SHOOTER.inverse();

    public static final Transform3d SHOOTER_TO_RIGHT_CAMERA = RIGHT_CAMERA_TO_SHOOTER.inverse();

    public static final Transform3d ROBOT_TO_LEFT_CAMERA = new Transform3d(
        new Translation3d(
            Inches.of(6.5),
            Inches.of(13.352),
            Inches.of(15.378)),
        new Rotation3d(
            Degrees.of(0),
            Degrees.of(30),
            Degrees.of(90)));

    public static final Transform3d ROBOT_TO_RIGHT_CAMERA = new Transform3d(
        new Translation3d(
            Inches.of(6.5),
            Inches.of(-13.352),
            Inches.of(15.378)),
        new Rotation3d(
            Degrees.of(0),
            Degrees.of(30),
            Degrees.of(-90)));
  
     public static final Transform3d ROBOT_TO_LEFT_SHOOTER = ROBOT_TO_LEFT_CAMERA.plus(LEFT_CAMERA_TO_SHOOTER);
    public static final Transform3d ROBOT_TO_RIGHT_SHOOTER = ROBOT_TO_RIGHT_CAMERA.plus(RIGHT_CAMERA_TO_SHOOTER);
      
    public static final Transform3d LEFT_CAMERA_TO_ROBOT = ROBOT_TO_LEFT_CAMERA.inverse();
    public static final Transform3d RIGHT_CAMERA_TO_ROBOT = ROBOT_TO_RIGHT_CAMERA.inverse();

    public static final int BLUE_ALLIANCE_HUB_TAG = 26;
    public static final int RED_ALLIANCE_HUB_TAG = 10;

    public static final Transform2d BLUE_ALLIANCE_TAG_TO_HUB = new Transform2d(
        Feet.of(1),
        Feet.of(0),
        new Rotation2d(Degrees.of(0)));

    public static final Transform2d RED_ALLIANCE_TAG_TO_HUB = new Transform2d(
        Feet.of(-1),
        Feet.of(0),
        new Rotation2d(Degrees.of(0)));

      public static final Function<PhotonPipelineResult, Matrix<N3, N1>> DEFAULT_VISIONSTDEV = x -> MatBuilder.fill(Nat.N3(), Nat.N1(), new double[]{0.3, 0.3, Radian.convertFrom(999999999, Degrees)});

  }

  public static class Climb {
    public static final String CANBUS_RIO = "rio";
    public static final int MOTOR_ID = -1;
    public static final double KP = -1;
    public static final double KI = -1;
    public static final double KD = -1;
    public static final double SENSOR_TO_MECHANISM_RATIO = -1;
    public static final double SUPPLY_CURRENT_LIMIT = -1;
    public static final double MAX_EXTN_POSITION = -1;
    public static final double MIN_EXTN_POSITION = -1;
    public static final double HOLD_POSITION = -1;
    public static final double POSITION_TOLERANCE = 0.1; // rotations (placeholder)
  }

  public static class Hopper {
    public static final int ROLLER_MOTOR_ID = 21;
    public static final double ROLLER_FORWARD_SPEED = 0.8;
    public static final double ROLLER_REVERSE_SPEED = -0.5;
  }

  public static class Shooter {
    public static final int TOP_FLYWHEEL_ID = 13;
    public static final int BOTTOM_FLYWHEEL_ID = 14;
    public static final int SHOOTER_PIVOT_ID = 11;

    public static final int SHOOTER_PIVOT_ENCODER = 18;

    public static final double FLYWHEEL_kP = 0.12448;
    public static final double FLYWHEEL_kI = 0.0;
    public static final double FLYWHEEL_kD = 0.000;
    public static final double FLYWHEEL_kV = 0.12;

    public static final double HOOD_ANGLE_KP = 50.0;
    public static final double HOOD_ANGLE_KI = 0.0;
    public static final double HOOD_ANGLE_KD = 0.000;

    public static final double FLYWHEEL_DIAMETER_INCHES = 4.0;
    public static final double FLYWHEEL_CIRCUMFERENCE_FT = Math.PI * FLYWHEEL_DIAMETER_INCHES / 12.0;

    public static final double FLYWHEEL_GEAR_RATIO = 1.0;
    public static final double FLYWHEEL_FPS_TOLERANCE = 2.0;
    public static final double SHOOTER_PIVOT_TOLERANCE = 1.5;
    public static final double SHOOTER_PIVOT_GEAR_RATIO = 50.0;  // 50:1 reduction

    public static final double LEFT_HOOD_MIN_ANGLE = 15.0;
    public static final double LEFT_HOOD_MAX_ANGLE = 45.0;
    public static final double LEFT_HOOD_BASE_ANGLE = 30.0;  // Center of left range

    public static final double RIGHT_HOOD_MIN_ANGLE = 135.0;
    public static final double RIGHT_HOOD_MAX_ANGLE = 165.0;
    public static final double RIGHT_HOOD_BASE_ANGLE = 150.0;  // Center of right range

    public static final double HOOD_ABSOLUTE_MIN = 10.0;
    public static final double HOOD_ABSOLUTE_MAX = 170.0;


    

  }
}
