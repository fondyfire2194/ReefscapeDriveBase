// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LedStrip;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindCurrentReefZone extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  Pose2d robotPose;
  double robotX;
  double robotY;
  double robotHeading;
  double yLimitForXHGZone;
  double plusYBorder;
  double minusYBorder;
  boolean zoneFound;
  int tst;
  LedStrip m_ledStrip;

  public FindCurrentReefZone(SwerveSubsystem swerve, LedStrip ledStrip) {
    m_swerve = swerve;
    m_ledStrip = ledStrip;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putBoolean("BLUERUNNING", true);
    zoneFound = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    zoneFound = false;
    m_swerve.reefZone = 0;
    m_swerve.reefZoneTag = 0;
    robotPose = m_swerve.getPose();
    robotX = robotPose.getX();
    robotY = robotPose.getY();
    robotHeading = robotPose.getRotation().getDegrees();

    if (m_swerve.isBlueAlliance()) {

      if (!zoneFound && robotX < FieldConstants.FIELD_LENGTH / 2 && checkBlueGHZone()) {
        m_swerve.reefZoneTag = 21;
        zoneFound = true;
        m_swerve.reefZone = 4;
      }
      if (checkBlueABZone()) {
        m_swerve.reefZoneTag = 18;
        zoneFound = true;
        m_swerve.reefZone = 1;
      }

      if (!zoneFound && checkBlueCDZone()) {
        m_swerve.reefZoneTag = 17;
        zoneFound = true;
        m_swerve.reefZone = 6;
      }

      if (!zoneFound && checkBlueEFZone()) {
        m_swerve.reefZoneTag = 22;
        zoneFound = true;
        m_swerve.reefZone = 5;

      }
      if (!zoneFound && checkBlueIJZone()) {
        m_swerve.reefZoneTag = 20;
        zoneFound = true;
        m_swerve.reefZone = 3;
      }

      if (!zoneFound && checkBlueKLZone()) {
        m_swerve.reefZoneTag = 19;
        zoneFound = true;
        m_swerve.reefZone = 2;
      }

      m_swerve.plusBorderPose = new Pose2d(robotX, plusYBorder, new Rotation2d());
      m_swerve.minusBorderPose = new Pose2d(robotX, minusYBorder, new Rotation2d());
    }
    // Red Alliance
    else {

      if (!zoneFound && robotX > FieldConstants.FIELD_LENGTH / 2 && checkRedGHZone()) {
        m_swerve.reefZoneTag = 10;
        zoneFound = true;
        m_swerve.reefZone = 4;
      }

      if (!zoneFound && checRedkABZone()) {
        m_swerve.reefZoneTag = 7;
        zoneFound = true;
        m_swerve.reefZone = 1;

      }
      if (!zoneFound && checkRedCDZone()) {
        m_swerve.reefZoneTag = 8;
        zoneFound = true;
        m_swerve.reefZone = 2;
      }

      if (!zoneFound && checkRedEFZone()) {
        m_swerve.reefZoneTag = 9;
        zoneFound = true;
        m_swerve.reefZone = 3;
      }
      if (!zoneFound && checkRedIJZone()) {
        m_swerve.reefZoneTag = 11;
        zoneFound = true;
        m_swerve.reefZone = 5;
      }

      if (!zoneFound && checkRedKLZone()) {
        m_swerve.reefZoneTag = 6;
        zoneFound = true;
        m_swerve.reefZone = 6;
      }

      m_swerve.plusBorderPose = new Pose2d(robotX, plusYBorder, new Rotation2d());
      m_swerve.minusBorderPose = new Pose2d(robotX, minusYBorder, new Rotation2d());
    }

    m_swerve.lockPoseChange = true;
    m_swerve.reefTargetPose = m_swerve.getTagPose(m_swerve.reefZoneTag).toPose2d();
    m_swerve.lockPoseChange = false;


    m_ledStrip.setViewOneColor(m_swerve.reefZone);
  }

  boolean checkBlueABZone() {
    plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        + (FieldConstants.blueReefABEdgeFromFieldOrigin - robotX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));
    minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        - (FieldConstants.blueReefABEdgeFromFieldOrigin - robotX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));

    boolean borderX = robotX < FieldConstants.blueReefABEdgeFromFieldOrigin;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkBlueGHZone() {

    plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        + (robotX - FieldConstants.blueReefGHEdgeFromFieldOrigin) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));
    minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        - (robotX - FieldConstants.blueReefGHEdgeFromFieldOrigin) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));

    boolean borderX = robotX < FieldConstants.FIELD_LENGTH / 2
        && robotX > FieldConstants.blueReefGHEdgeFromFieldOrigin;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkBlueCDZone() {
    return robotX < FieldConstants.blueReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkBlueEFZone() {
    return robotX < FieldConstants.FIELD_LENGTH / 2 && robotX > FieldConstants.blueReefMidFromCenterFieldX
        && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkBlueIJZone() {
    return robotX < FieldConstants.FIELD_LENGTH / 2 && robotX > FieldConstants.blueReefMidFromCenterFieldX
        && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkBlueKLZone() {
    return robotX < FieldConstants.blueReefMidFromCenterFieldX
        && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkRedGHZone() {
    plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        + (FieldConstants.redReefGHEdgeFromCenterFieldX - robotX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));
    minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        - (FieldConstants.redReefGHEdgeFromCenterFieldX - robotX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));

    boolean borderX = robotX > FieldConstants.FIELD_LENGTH / 2
        && robotX < FieldConstants.redReefGHEdgeFromCenterFieldX;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checRedkABZone() {

    plusYBorder = FieldConstants.FIELD_WIDTH / 2 + FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        + (robotX - FieldConstants.redReefABEdgeFromCenterFieldX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));
    minusYBorder = FieldConstants.FIELD_WIDTH / 2 - FieldConstants.reefSideWidth / FieldConstants.reefSideWidthDiv
        - (robotX - FieldConstants.redReefABEdgeFromCenterFieldX) *
            Math.tan(Units.degreesToRadians(m_swerve.yZoneLimitAngle));

    boolean borderX = robotX > FieldConstants.redReefABEdgeFromCenterFieldX;

    return borderX
        && (robotY < plusYBorder &&
            robotY > minusYBorder);
  }

  boolean checkRedCDZone() {
    return robotX > FieldConstants.redReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkRedEFZone() {
    return robotX > FieldConstants.FIELD_LENGTH / 2 && robotX < FieldConstants.redReefMidFromCenterFieldX
        && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkRedIJZone() {
    return robotX > FieldConstants.FIELD_LENGTH / 2 && robotX < FieldConstants.redReefMidFromCenterFieldX
        && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkRedKLZone() {
    return robotX > FieldConstants.redReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
