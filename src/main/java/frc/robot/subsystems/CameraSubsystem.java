// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CameraType;

public class CameraSubsystem extends SubsystemBase {

  CameraType cameraType;
  Pose3d pose = new Pose3d();
  String cameraName = "";
  PhotonCamera photonCamera;
  List<PhotonTrackedTarget> targets = result.getTargets();
  PhotonPipelineResult result;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem(CameraType cameraType, String cameraName) {
    this.cameraType = cameraType;
    this.cameraName = cameraName;

    if(cameraType == CameraType.PHOTONVISION){
      photonCamera = new PhotonCamera(cameraName);
    }
    else{

    }
  }

  public boolean hasTargets(){
    return result.hasTargets();
  }

  public CameraType getCameraType(){
    return cameraType;
  }

  public Pose3d getPose(){
    if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
      pose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
    }
    return this.pose;
  }

  @Override
  public void periodic() {
    
  }
}
