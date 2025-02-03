// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OverridePathToTag extends Command {
  /** Creates a new OverridePathToTag. */
  private final SwerveSubsystem m_swerve;
  private String m_camName;
  private int m_tagNumber;
  double tagAngle;
  double correctionXLimit = 1;
  double kpx = 0;
  double correctionYLimit = 1;
  double kpy = 0;
  double correctionRLimit = Math.toRadians(1);

  double kpr = 0;

  PathPlannerTrajectory m_traj;

  public OverridePathToTag(SwerveSubsystem swerve, String camName, int tagNumber, PathPlannerTrajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_camName = camName;
    m_tagNumber = tagNumber;
    m_traj=traj;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPriorityTagID(m_camName, m_tagNumber);
    tagAngle = Math.PI - m_swerve.aprilTagFieldLayout.getTagPose(m_tagNumber).get().getRotation().getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_traj.getState(3).pose.getX();
    double tx = LimelightHelpers.getTX(m_camName);
    if (Math.abs(tx) > correctionXLimit) {
      PPHolonomicDriveController.overrideXFeedback(() -> tx * kpx);
    }
    double ty = LimelightHelpers.getTY(m_camName);
    if (Math.abs(ty) > correctionYLimit) {
      PPHolonomicDriveController.overrideYFeedback(() -> ty * kpy);
    }
    PPHolonomicDriveController
        .overrideRotationFeedback(() -> (tagAngle - m_swerve.getPose().getRotation().getRadians()) * kpr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PPHolonomicDriveController.clearFeedbackOverrides();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
