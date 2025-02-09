// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAlgaeProcessor extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  Pose2d robotPose;
  double robotX;
  double robotY;
  double robotHeading;

  boolean exit;
  int tst;

  public DriveToAlgaeProcessor(SwerveSubsystem swerve) {
    m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    exit = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("TEST", tst++);
    exit = false;

    robotX = m_swerve.getPose().getX();
    robotY = m_swerve.getPose().getY();

    if (m_swerve.isRedAlliance() && robotX > FieldConstants.FIELD_LENGTH / 2) {
      m_swerve.processorStationTag = 3;
    }

    if (m_swerve.isBlueAlliance() && robotX < FieldConstants.FIELD_LENGTH / 2) {
      m_swerve.processorStationTag = 16;
    }

    int tagNumber = m_swerve.processorStationTag;

    m_swerve.processorStationTargetPose = m_swerve.getTagPose(tagNumber).toPose2d();

    double baseOffset = RobotConstants.algaeOffset + RobotConstants.ROBOT_LENGTH / 2;

    Translation2d tl2d = new Translation2d(baseOffset, 0);

    Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));

    m_swerve.processorStationFinalTargetPose = m_swerve.processorStationTargetPose.transformBy(tr2d);

    m_swerve.driveToPose(m_swerve.processorStationFinalTargetPose).schedule();

    exit = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exit;
  }
}
