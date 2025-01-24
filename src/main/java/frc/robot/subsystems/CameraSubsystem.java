// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CameraType;

public class CameraSubsystem extends SubsystemBase {

  CameraType cameraType;
  Pose3d pose = new Pose3d();
  String cameraName = "";
  PhotonCamera photonCamera;
  List<PhotonPipelineResult> resultList;
  PhotonTrackedTarget bestResult;
  PhotonUtils utils;
  AprilTagFieldLayout aprilTagFieldLayout;
  Transform3d bestCameraToTarget;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem(CameraType cameraType, String cameraName) {
    this.cameraType = cameraType;
    this.cameraName = cameraName;

    if(cameraType == CameraType.PHOTONVISION){
      photonCamera = new PhotonCamera(cameraName);
      resultList = photonCamera.getAllUnreadResults();
      if(!resultList.isEmpty()){
        bestResult = resultList.get(0).getBestTarget();
        bestCameraToTarget = bestResult.getBestCameraToTarget();
      }
    }
    else{

    }
  }

  public boolean hasTargets(){
    return resultList.get(0).hasTargets();
  }

  public CameraType getCameraType(){
    return cameraType;
  }

  public Pose3d getPose(){
    if (aprilTagFieldLayout.getTagPose(bestResult.getFiducialId()).isPresent()) {
      pose = PhotonUtils.estimateFieldToRobotAprilTag(bestResult.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(bestResult.getFiducialId()).get(), bestCameraToTarget);
    }
    return this.pose;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pose X", this.getPose().getX());
    SmartDashboard.putNumber("Pose Y", this.getPose().getY());
    SmartDashboard.putNumber("Pose T", this.getPose().getRotation().getAngle());
  }
}
