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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CameraType;

public class CameraSubsystem extends SubsystemBase {

  CameraType cameraType;
  Pose3d pose3d = new Pose3d();
  String cameraName = "";
  PhotonCamera photonCamera;
  List<PhotonPipelineResult> resultList;
  PhotonTrackedTarget target;
  PhotonUtils utils;
  AprilTagFieldLayout aprilTagFieldLayout;
  Transform3d bestCameraToTarget;
  DoubleLogEntry logPoseX, logPoseY, logPoseT;
  Pose2d currentPose = new Pose2d();
  PhotonPipelineResult result;
  Transform3d cameraToRobotPose = new Transform3d();

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem(CameraType cameraType, String cameraName) {
    this.cameraType = cameraType;
    this.cameraName = cameraName;
    DataLog log = DataLogManager.getLog();
    logPoseX = new DoubleLogEntry(log, "/U/poseX");
    logPoseY = new DoubleLogEntry(log, "/U/poseY");
    logPoseT = new DoubleLogEntry(log, "/U/poseT");

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    if(cameraType == CameraType.PHOTONVISION){
      photonCamera = new PhotonCamera(cameraName);
      
    }
    else{

    }
  }
  public void updateResults() {
 
  }

  public CameraType getCameraType(){
    return cameraType;
  }

  public Pose3d update3DPose(){
    System.out.println(aprilTagFieldLayout.getTagPose(target.getFiducialId()));
    if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
      pose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobotPose);
      System.out.println("Present");
    }
    //System.out.println(pose3d);
    return this.pose3d;
  }

  public Pose2d update2DPose(){
    return update3DPose().toPose2d();
  }

  public Pose2d getPose2d(){
    return currentPose;
  }

  @Override
  public void periodic() {
      result = photonCamera.getLatestResult();
      target = result.getBestTarget();
      if(result.hasTargets()){
        currentPose = update2DPose();
        
        SmartDashboard.putNumber("poseX", currentPose.getX());
        SmartDashboard.putNumber("poseY", currentPose.getY());
        SmartDashboard.putNumber("poseR", currentPose.getRotation().getDegrees());
        // logPoseX.append(currentPose.getX());
        // logPoseY.append(currentPose.getY());
        // logPoseT.append(currentPose.getRotation().getDegrees());
      }
    
    
  }
}
