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
public class FindCurrentReefZoneBlue extends Command {
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
  public FindCurrentReefZoneBlue(SwerveSubsystem swerve,LedStrip ledStrip ) {
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

    if (checkABZone()) {
      m_swerve.reefZoneTag = 18;
      zoneFound = true;
      m_swerve.reefZone=1;
    }
    if (!zoneFound && robotX < FieldConstants.FIELD_LENGTH / 2 && checkGHZone()) {
      m_swerve.reefZoneTag = 21;
      zoneFound = true;
      m_swerve.reefZone=4;
    }

    if (!zoneFound && checkCDZone()) {
      m_swerve.reefZoneTag = 17;
      zoneFound = true;
      m_swerve.reefZone=6;
    }

    if (!zoneFound && checkEFZone()) {
      m_swerve.reefZoneTag = 22;
      zoneFound = true;
      m_swerve.reefZone=5;
    }
    if (!zoneFound && checkIJZone()) {
      m_swerve.reefZoneTag = 20;
      zoneFound = true;
      m_swerve.reefZone=3;
    }

    if (!zoneFound && checkKLZone()) {
      m_swerve.reefZoneTag = 19;
      zoneFound = true;
      m_swerve.reefZone=2;
    }

    m_swerve.plusBorderPose = new Pose2d(robotX, plusYBorder, new Rotation2d());
    m_swerve.minusBorderPose = new Pose2d(robotX, minusYBorder, new Rotation2d());
    
    m_swerve.lockPoseChange = true;
    m_swerve.reefTargetPose = m_swerve.getTagPose(m_swerve.reefZoneTag).toPose2d();
    m_swerve.lockPoseChange = false;

    m_ledStrip.setViewOneColor(m_swerve.reefZone);
  }

  boolean checkABZone() {
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

  boolean checkGHZone() {

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

  boolean checkCDZone() {
    return robotX < FieldConstants.blueReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkEFZone() {
    return robotX < FieldConstants.FIELD_LENGTH / 2 && robotX > FieldConstants.blueReefMidFromCenterFieldX && robotY < FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkIJZone() {
    return robotX < FieldConstants.FIELD_LENGTH / 2 && robotX > FieldConstants.blueReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
  }

  boolean checkKLZone() {
    return robotX < FieldConstants.blueReefMidFromCenterFieldX && robotY > FieldConstants.FIELD_WIDTH / 2;
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
