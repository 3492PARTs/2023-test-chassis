// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.cameraSystem;
import frc.robot.subsystems.driveTrain;

public class updatePoseVisually extends CommandBase {
  driveTrain dtrain; 
  cameraSystem cSystem;

  /** Creates a new updatePoseVisually. */
  public updatePoseVisually() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dtrain = driveTrain.getInstance();
    cSystem = cameraSystem.getCameraSystem();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Pair<Pose2d, Double> pose = cSystem.getEstimatedGlobalPose(dtrain.currentPose());
    if(pose.getFirst() == null){
    }
    else{
      dtrain.updatePoseVisually(pose);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
