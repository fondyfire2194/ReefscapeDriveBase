// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
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
public final class VisionConstants {


  public static final class CameraConstants {

    public static class CameraValues {
      public String camname = "name";
      public String ipaddress = "ip";
      public double forward;
      public double side;
      public double up;
      public double roll;
      public double pitch;
      public double yaw;
      public double hfov;
      public double vfov;
      public int horpixels;
      public int vertpixels;
      public boolean isUsed = true;
      public boolean isActive = false;

      public CameraValues(
          final String camname,
          final String ipaddress,
          final double forward, final double side, final double up, final double roll,
          final double pitch, final double yaw,
          final double hfov, double vfov,
          final int horpixels, final int vertpixels,
          final boolean isUsed,
          final boolean isActive) {
        this.camname = camname;
        this.ipaddress = ipaddress;
        this.forward = forward;
        this.side = side;
        this.up = up;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
        this.hfov = hfov;
        this.vfov = vfov;
        this.horpixels = horpixels;
        this.vertpixels = vertpixels;
        this.isUsed = isUsed;
        this.isActive = isActive;
      }
    }

    public static CameraValues frontLeftCamera = new CameraValues(
        "limelight-frleft",
        "10.21.94.15",
        Units.inchesToMeters(14),
        Units.inchesToMeters(-8),
        Units.inchesToMeters(8.5),
        0,
        12, // deg 19
        7.5,
        63.3,
        49.7,
        1,
        1,
        true,
        false);

    public static CameraValues frontRightCamera = new CameraValues(
        "limelight-frright",
        "10.21.94.16",
        Units.inchesToMeters(14),
        Units.inchesToMeters(8),
        Units.inchesToMeters(8.5),
        0,
        12, // deg
        -7.5,
        63.3,
        49.7,
        1,
        1,
        true,
        false);

    public static CameraValues rearCamera = new CameraValues(
        "limelight-rear",
        "10.21.94.17",
        Units.inchesToMeters(0),
        Units.inchesToMeters(-17.25),
        Units.inchesToMeters(9.0),
        0,
        5,
        0,
        63.3,
        49.7,
        1280,
        960,
        true,
        false);

    public static final double POSE_AMBIGUITY_CUTOFF = 0.05;
    public static final double DISTANCE_CUTOFF = 4.0;

  }

}
