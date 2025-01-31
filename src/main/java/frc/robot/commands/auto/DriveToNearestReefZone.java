// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestReefZone extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;

  boolean exit;
  int tst;
  Side m_side;

  Translation2d tl2d;

  public DriveToNearestReefZone(SwerveSubsystem swerve, Side side) {
    m_swerve = swerve;
    m_side = side;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tst = 0;
    exit = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = new Pose2d();

    if (!m_swerve.lockPoseChange) {

      targetPose = m_swerve.reefTargetPose;

      double baseOffset = RobotConstants.placementOffset + RobotConstants.ROBOT_LENGTH / 2;

      if (m_side == Side.CENTER)
        tl2d = new Translation2d(baseOffset, 0);
      if (m_side == Side.RIGHT)
        tl2d = new Translation2d(baseOffset, FieldConstants.reefOffset);
      if (m_side == Side.LEFT)
        tl2d = new Translation2d(baseOffset, -FieldConstants.reefOffset);

      Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));

      m_swerve.reefFinalTargetPose = targetPose.transformBy(tr2d);

      m_swerve.driveToPose(m_swerve.reefFinalTargetPose).schedule();

      exit = true;
    } else
      tst++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exit || tst > 2;
  }
}
