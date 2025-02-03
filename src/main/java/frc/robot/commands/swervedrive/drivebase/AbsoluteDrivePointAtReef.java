// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * A more advanced Swerve Control System that has 4 buttons for which direction
 * to face
 */
public class AbsoluteDrivePointAtReef extends Command {

  private final SwerveSubsystem swerve;
  private final DoubleSupplier vX, vY;
  private final DoubleSupplier headingAdjust;
  private boolean resetHeading = false;
  private double[] zoneHeadingXBlue = { 0, 0, -0.866, -.866, 0, .866, 0.866 };
  private double[] zoneHeadingYBlue = { 0, 1, .5, -.5, -1, -.5, 0.5 };

  private double[] zoneHeadingXRed = { 0, 0, -.866, -.866, 0, .866, .866 };
  private double[] zoneHeadingYRed = { 0, -1, -.5, .5, 1, .5, -.5 };
  private int reefZoneLast = 9;
  private int ctr;
  double headingX = 0;
  double headingY = 0;

  /**
   * Used to drive a swerve robot in full field-centric mode. vX and vY supply
   * translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. Heading Adjust changes
   * the current heading after being
   * multipied by a constant. The look booleans are shortcuts to get the robot to
   * face a certian direction. Based off of
   * ideas in
   * https://www.chiefdelphi.com/t/experiments-with-a-swerve-steering-knob/446172
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick
   *                      input. Should be in the range -1 to 1
   *                      with deadband already accounted for. Positive X is away
   *                      from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick
   *                      input. Should be in the range -1 to 1
   *                      with deadband already accounted for. Positive Y is
   *                      towards the left wall when looking through
   *                      the driver station glass.
   * @param headingAdjust DoubleSupplier that supplies the component of the
   *                      robot's heading angle that should be
   *                      adjusted. Should range from -1 to 1 with deadband
   *                      already accounted for.
   *
   */
  public AbsoluteDrivePointAtReef(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
      DoubleSupplier headingAdjust) {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.headingAdjust = headingAdjust;

    addRequirements(this.swerve);
  }

  @Override
  public void initialize() {
    resetHeading = true;
    reefZoneLast = 9;
    ctr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("HEADINGZONE", swerve.reefZone);
    SmartDashboard.putNumber("HEADINGZONELAST", reefZoneLast);

    if (!resetHeading && swerve.reefZone != 0 && swerve.reefZone != reefZoneLast) {

      if (swerve.isBlueAlliance()) {

        headingX = zoneHeadingXBlue[swerve.reefZone];
        headingY = zoneHeadingYBlue[swerve.reefZone];

      }

      else {
        headingX = zoneHeadingXRed[swerve.reefZone];
        headingY = zoneHeadingYRed[swerve.reefZone];

      }
      ctr++;
      SmartDashboard.putNumber("HEADINGCT", ctr);
      SmartDashboard.putNumber("HEADINGX", headingX);
      SmartDashboard.putNumber("HEADINGY", headingY);
      SmartDashboard.putNumber("HEADINGADJUST", Math.abs(headingAdjust.getAsDouble()));
      reefZoneLast = swerve.reefZone;
    }

    // Prevent Movement After Auto
    if (resetHeading) {
      if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) == 0) {
        // Get the curret Heading
        Rotation2d currentHeading = swerve.getHeading();

        // Set the Current Heading to the desired Heading
        headingX = currentHeading.getSin();
        headingY = currentHeading.getCos();
      }
      // Dont reset Heading Again
      resetHeading = false;
    }

    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

    // Limit velocity to prevent tippy
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    // Make the robot move
    if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
      resetHeading = true;
      swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
    } else {
      swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }
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
