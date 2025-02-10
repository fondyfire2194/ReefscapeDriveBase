package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPose extends Command {

    private final SwerveSubsystem drivetrain;

    private Pose2d targetPose;

    private ProfiledPIDController xPidController, yPidController, driverRotationPidController;

    public DriveToPose(SwerveSubsystem drivetrainSubsystem, Pose2d targetPoseSupplier) {
        drivetrain = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {

        xPidController = new ProfiledPIDController(2.5, 0., 0,
                new TrapezoidProfile.Constraints(3, 6));
        yPidController = new ProfiledPIDController(2.5, 0., 0,
                new TrapezoidProfile.Constraints(3, 6));

        driverRotationPidController = new ProfiledPIDController(10, 0., 0,
                new TrapezoidProfile.Constraints(Math.PI, Math.PI));
        driverRotationPidController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateChassisSpeeds(drivetrain.getPose(), targetPose);
        drivetrain.drive(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

    public ChassisSpeeds calculateChassisSpeeds(Pose2d currentPose, Pose2d targetPose) {
        double xFeedback = xPidController.calculate(currentPose.getX(), targetPose.getX());
        double yFeedback = yPidController.calculate(currentPose.getY(), targetPose.getY());
        double thetaFeedback = calculateHeadingPID(currentPose.getRotation(), targetPose.getRotation().getDegrees());

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFeedback, yFeedback, thetaFeedback, currentPose.getRotation());
    }

    public double calculateHeadingPID(Rotation2d heading, double targetDegrees) {
        double headingDegrees = heading.getDegrees();
        double headingError = targetDegrees - headingDegrees;

        return driverRotationPidController.calculate(
                heading.getRadians(),
                Math.toRadians(targetDegrees));
    }

}
