// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public final class CANIDConstants {

    /**
     * Drive and angle constants are for reference only
     * They are specigfied in the YAGSL module JSON filkes
     */
    public static final int flDrive = 3;
    public static final int frDrive = 4;
    public static final int blDrive = 5;
    public static final int brDrive = 6;

    public static final int flAngle = 7;
    public static final int frAngle = 8;
    public static final int blAngle = 9;
    public static final int brAngle = 10;

    public static final int leftElevatorID = 20;
    public static final int rightElevatorID = 21;
    public static final int armMotorID = 22;
    public static final int algaeintakeID = 23;
    public static final int coralintakeID = 24;

  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class FieldConstants {
    public static final double FIELD_LENGTH = Units.inchesToMeters(690.875);
    public static final double FIELD_WIDTH = Units.inchesToMeters(317);
    public static final double redReefGHEdgeFromCenterFieldX = Units.inchesToMeters(472);
    public static final double redReefABEdgeFromCenterFieldX = Units.inchesToMeters(570);
    public static final double redReefMidFromCenterFieldX = FIELD_LENGTH - Units.inchesToMeters(144)
        - Units.inchesToMeters(65.25 / 2);

    public static final double blueReefABEdgeFromFieldOrigin = Units.inchesToMeters(144);
    public static final double blueReefGHEdgeFromFieldOrigin = Units.inchesToMeters(144 + 65.5);
    public static final double blueReefMidFromCenterFieldX = Units.inchesToMeters(144 + 65.5 / 2);

    public static final double reefSideWidth = Units.inchesToMeters(37.82);
    public static final double reefSideWidthDiv = 4;

    public static final double reefOffset = Units.inchesToMeters(6.5);
    public static final double centerToReefBranch = Units.inchesToMeters(13);

    public static int[] blueReefTags = { 0, 21, 22, 17, 18, 19, 20 };
    public static int[] blueCoralStationTags = { 13, 12 };
    public static int blueProcessorTag = 16;
    public static int[] redReefTags = { 0, 10, 11, 6, 7, 8, 9 };
    public static int[] redCoralStationTags = { 2, 1 };
    public static int redProcessorTag = 3;

    public static enum Side {
      LEFT,
      CENTER,
      RIGHT
    }
  }

  public static class RobotConstants {
    public static final double ROBOT_LENGTH = Units.inchesToMeters(28);
    public static final double placementOffset = Units.inchesToMeters(12);
    public static final double pickupOffset = Units.inchesToMeters(8);
    public static double algaeOffset = 6;

  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 10;

    // public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass = 4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    // public static final double kMinElevatorHeightMeters = 0.922; // m
    // public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads = Units.degreesToRadians(-40); // -50.1 deg from horiz
    public static final double kMaxAngleRads = Units.degreesToRadians(40.9); // 40.9 deg from horiz

  }
}
