// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.LimelightHelpers;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  private double llHeartbeatfl;
  private double llHeartbeatLastfl;
  private int samplesfl;

  public boolean limelightExistsfl;

  private double llHeartbeatfr;
  private double llHeartbeatLastfr;
  private int samplesfr;

  public boolean limelightExistsfr;

  private double llHeartbeatr;
  private double llHeartbeatLastr;
  private int samplesr;

  public boolean limelightExistsr;
  private int loopctr;

  public String flname = VisionConstants.CameraConstants.frontLeftCamera.camname;
  public String frname = VisionConstants.CameraConstants.frontRightCamera.camname;
  public String rname = VisionConstants.CameraConstants.rearCamera.camname;

  Optional<Pose3d> temp;

  final int[] autoTagFilter = new int[] { 10, 11, 6, 7, 8, 9, 21, 22, 17, 18, 19, 20 };

  Alert flCameraAlert = new Alert("FrontLeftCameraProblem", AlertType.kWarning);
  Alert frCameraAlert = new Alert("FrontRightCameraProblem", AlertType.kError);
  Alert rearCameraAlert = new Alert("RearCameraProblem", AlertType.kInfo);

  public LimelightVision() {

    if (VisionConstants.CameraConstants.frontLeftCamera.isUsed) {
      setCamToRobotOffset(VisionConstants.CameraConstants.frontLeftCamera);
    }

    if (VisionConstants.CameraConstants.frontRightCamera.isUsed)
      setCamToRobotOffset(VisionConstants.CameraConstants.frontRightCamera);

  }

  public void setAprilTagFilter(String camname) {
    LimelightHelpers.SetFiducialIDFiltersOverride(camname, autoTagFilter);
  }

  public void setPOIRight(String camname) {
    LimelightHelpers.SetFidcuial3DOffset(camname, FieldConstants.centerToReefBranch, 0, 0);
  }

  public void setPOILeft(String camname) {
    LimelightHelpers.SetFidcuial3DOffset(camname, FieldConstants.centerToReefBranch, 0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    flCameraAlert.set(limelightExistsfl == false);
    frCameraAlert.set(limelightExistsfr == false);
    rearCameraAlert.set(limelightExistsr == false);

    if (loopctr > 2)
      loopctr = 0;

    if (RobotBase.isReal()) {
      if (VisionConstants.CameraConstants.frontLeftCamera.isUsed && loopctr == 0) {
        llHeartbeatfl = LimelightHelpers.getLimelightNTDouble(VisionConstants.CameraConstants.frontLeftCamera.camname,
            "hb");
        if (llHeartbeatfl == llHeartbeatLastfl) {
          samplesfl += 1;
        } else {
          samplesfl = 0;
          llHeartbeatLastfl = llHeartbeatfl;
          limelightExistsfl = true;
        }
        if (samplesfl > 5)
          limelightExistsfl = false;

        VisionConstants.CameraConstants.frontLeftCamera.isActive = limelightExistsfl;
      }
      if (VisionConstants.CameraConstants.frontRightCamera.isUsed && loopctr == 1) {
        llHeartbeatfr = LimelightHelpers.getLimelightNTDouble(VisionConstants.CameraConstants.frontRightCamera.camname,
            "hb");
        if (llHeartbeatfr == llHeartbeatLastfr) {
          samplesfr += 1;
        } else {
          samplesfr = 0;
          llHeartbeatLastfr = llHeartbeatfr;
          limelightExistsfr = true;
        }
        if (samplesfr > 5)
          limelightExistsfr = false;

        VisionConstants.CameraConstants.frontRightCamera.isActive = limelightExistsfr;
      }

      if (VisionConstants.CameraConstants.rearCamera.isUsed && loopctr == 2) {
        llHeartbeatr = LimelightHelpers.getLimelightNTDouble(VisionConstants.CameraConstants.rearCamera.camname, "hb");
        if (llHeartbeatr == llHeartbeatLastr) {
          samplesr += 1;
        } else {
          samplesr = 0;
          llHeartbeatLastr = llHeartbeatr;
          limelightExistsr = true;
        }
        if (samplesr > 5)
          limelightExistsr = false;

        VisionConstants.CameraConstants.rearCamera.isActive = limelightExistsr;
      }
    }

    loopctr++;

    SmartDashboard.putBoolean("LL//FrontLeftCamOk", limelightExistsfl);
    SmartDashboard.putBoolean("LL//FrontRightCamOk", limelightExistsfr);
    SmartDashboard.putBoolean("LL//RearCamOk", limelightExistsr);

    boolean allcamsok = VisionConstants.CameraConstants.frontLeftCamera.isUsed && limelightExistsfl
        && VisionConstants.CameraConstants.frontRightCamera.isUsed && limelightExistsfr
        && VisionConstants.CameraConstants.rearCamera.isUsed && limelightExistsr;
    SmartDashboard.putBoolean("LL//CamsOK", allcamsok);
  }

  public void setCamToRobotOffset(VisionConstants.CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }

}