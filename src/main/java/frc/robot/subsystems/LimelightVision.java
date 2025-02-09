// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import monologue.Annotations.Log;
import monologue.Logged;

public class LimelightVision extends SubsystemBase implements Logged {
  /** Creates a new LimelightVision. */

  public boolean limelightExistsfl;

  public boolean limelightExistsfr;

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
  @Log(key = "flaccpose")
  public static Pose2d flAcceptedPose;
  @Log(key = "fraccpose")
  public static Pose2d frAcceptedPose;
  @Log(key = "rearaccpose")
  public static Pose2d rearAcceptedPose;
  @Log(key = "flaccepted")
  public static int flAcceptedCount;
  @Log(key = "fraccepted")
  public static int frAcceptedCount;
  @Log(key = "rearaccepted")
  public static int rearAcceptedCount;

  /**
   * Checks if the specified limelight is connected
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight network table contains the key "tv"
   */
  public boolean isLimelightConnected(String camname) {
    return LimelightHelpers.getLimelightNTTable(camname).containsKey("tv");
  }

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

  public double getDistanceToTag(String camname) {
    if (LimelightHelpers.getTV(camname)) {
      Pose3d tag3dpose = LimelightHelpers.getTargetPose3d_CameraSpace(camname);
      return tag3dpose.getTranslation().getNorm();
    } else
      return 0;
  }


  @Override
  public void periodic() {

    if (RobotBase.isReal()) {
      if (VisionConstants.CameraConstants.frontLeftCamera.isUsed && loopctr == 0) {
        limelightExistsfl = isLimelightConnected(CameraConstants.frontLeftCamera.camname);
        limelightExistsfr = isLimelightConnected(CameraConstants.frontRightCamera.camname);
        limelightExistsr = isLimelightConnected(CameraConstants.rearCamera.camname);

        boolean allcamsok = VisionConstants.CameraConstants.frontLeftCamera.isUsed && limelightExistsfl
            && VisionConstants.CameraConstants.frontRightCamera.isUsed && limelightExistsfr
            && VisionConstants.CameraConstants.rearCamera.isUsed && limelightExistsr;
        SmartDashboard.putBoolean("LL//CamsOK", allcamsok);
      }

      SmartDashboard.putBoolean("LL//FrontLeftCamOk", limelightExistsfl);
      SmartDashboard.putBoolean("LL//FrontRightCamOk", limelightExistsfr);
      SmartDashboard.putBoolean("LL//RearCamOk", limelightExistsr);
    }
  }

  public void setCamToRobotOffset(VisionConstants.CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }

}