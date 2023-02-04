// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class cameraSystem extends SubsystemBase {
  /** Creates a new cameraSystem. */

  AprilTagFieldLayout aprilTagFieldLayout;
  UsbCamera frontCamera = CameraServer.startAutomaticCapture(0);
  UsbCamera backCamera = CameraServer.startAutomaticCapture(1);
  VideoSink server = CameraServer.getServer();


    
  PhotonCamera cam = new PhotonCamera("OV5647");
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  PhotonPoseEstimator robotPoseEstimator;

  private static cameraSystem m_CameraSystem = new cameraSystem();
    // ... Add other cameras here

    // Assemble the list of cameras & mount locations
  ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();

  public static cameraSystem getCameraSystem(){
    return m_CameraSystem;
  }
    
  
  
  public cameraSystem() {
    camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));
    try {
      aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFields.kDefaultField.toString());
    
      } catch (Exception e) {
        // TODO: handle exception
      }
      robotPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, cam, robotToCam);

      CameraServer.addSwitchedCamera("directionalCamera");
      

     
    

  }

  public void swapCamera(String cameraDirection){
    VideoSource cameraToSwitchTo;
    if(cameraDirection.equals("front")){
      cameraToSwitchTo = frontCamera;
    }
    else if(cameraDirection.equals("back")){
      cameraToSwitchTo = backCamera;
    }
    else{
      cameraToSwitchTo = frontCamera;
    }

    CameraServer.getVideo("directionalCamera").setSource(cameraToSwitchTo);

    
  }


   

  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<EstimatedRobotPose> result = robotPoseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().estimatedPose.toPose2d(), currentTime - result.get().timestampSeconds);
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}