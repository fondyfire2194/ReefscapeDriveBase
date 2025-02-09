// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class LimelightTagsUpdate {

    private final SwerveSubsystem m_swerve;

    private final CameraConstants.CameraValues m_cam;
    private boolean m_useMegaTag2 = false;

    boolean rejectUpdate;
    private boolean logVision = true;

    private double lastYaw;

    public LimelightTagsUpdate(CameraConstants.CameraValues cam, SwerveSubsystem swerve) {
        m_cam = cam;
        m_swerve = swerve;

    }

    public void setUseMegatag2(boolean on) {
        m_useMegaTag2 = on;
    }

    public void setLLRobotorientation() {
        double yaw = m_swerve.getYaw().getDegrees();
        double yawRateChange = (lastYaw - yaw)*50;
        lastYaw = yaw;
        LimelightHelpers.SetRobotOrientation(m_cam.camname,
                yaw, yawRateChange, 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()
    }

    public void execute() {

        if (m_cam.isActive && !m_swerve.inhibitVision && LimelightHelpers.getTV(m_cam.camname)) {
            setLLRobotorientation();
            if (m_useMegaTag2) {
                setLLRobotorientation();
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cam.camname);
                m_swerve.distanceLimelightToEstimator = mt2.pose.getTranslation()
                        .getDistance(m_swerve.getPoseEstimator().getEstimatedPosition().getTranslation());

                rejectUpdate = mt2.tagCount == 0 || Math.abs(m_swerve.getGyroRate()) > 720
                        || (mt2.tagCount == 1 && mt2.rawFiducials.length == 1 &&
                                mt2.rawFiducials[0].ambiguity > .7
                                && mt2.rawFiducials[0].distToCamera > 5);

                if (!rejectUpdate) {

                    if (logVision) {

                        if (m_cam == CameraConstants.frontLeftCamera) {
                            LimelightVision.flAcceptedPose = mt2.pose;
                            LimelightVision.flAcceptedCount++;
                        }

                        if (m_cam == CameraConstants.frontRightCamera) {
                            LimelightVision.frAcceptedPose = mt2.pose;
                            LimelightVision.frAcceptedCount++;
                        }
                        if (m_cam == CameraConstants.rearCamera) {
                            LimelightVision.rearAcceptedPose = mt2.pose;
                            LimelightVision.rearAcceptedCount++;
                        }
                    }
                    m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(1.0,
                            1.0, 9999999));
                    m_swerve.getPoseEstimator().addVisionMeasurement(
                            mt2.pose,
                            mt2.timestampSeconds);
                }

            } else {

                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cam.camname);

                rejectUpdate = mt1.tagCount == 0
                        || mt1.tagCount == 1 && mt1.rawFiducials.length == 1 &&
                                mt1.rawFiducials[0].ambiguity > .7
                                && mt1.rawFiducials[0].distToCamera > 5;

                if (!rejectUpdate) {
                    m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7,
                            .7, 1));
                    m_swerve.getPoseEstimator().addVisionMeasurement(
                            mt1.pose,
                            mt1.timestampSeconds);
                }
            }
        }
    }
}
