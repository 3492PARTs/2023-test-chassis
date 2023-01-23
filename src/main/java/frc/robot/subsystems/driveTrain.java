// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import PARTSlib2023.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import PARTSlib2023.PARTS.frc.Utils.sensors.wheelLinearDistance;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrain extends beanieDriveTrain {


  wheelLinearDistance[] leftWheels = new wheelLinearDistance[2];
  wheelLinearDistance[] rightWheels = new wheelLinearDistance[2];

  DifferentialDriveKinematics dKinematics = new DifferentialDriveKinematics((Double) null); //tbd
  DifferentialDrivePoseEstimator dEstimator = new DifferentialDrivePoseEstimator(dKinematics, getRotation(), leftDistance(), rightDistance(), null);
  private static driveTrain mDriveTrain = new driveTrain();
    

  /** Creates a new driveTrain. */
  public driveTrain() {
    super(new AHRS() , new MotorControllerGroup(new CANSparkMax(0, MotorType.kBrushless),new CANSparkMax(0, MotorType.kBrushless)),
     new MotorControllerGroup(new CANSparkMax(0, MotorType.kBrushless ),new CANSparkMax(0, MotorType.kBrushless)));
  }

  @Override
  public beanieDriveTrain getInstance() {
      return mDriveTrain;
  }

  @Override
  public Pose2d currentPose() {
      return dEstimator.getEstimatedPosition();
  }

  @Override
  public double rightDistance() {
      // TODO Auto-generated method stub

      return wheelAverage(rightWheels);
  }


  private double wheelAverage(wheelLinearDistance[] wheelDistances){

    double average = 0;
    for(wheelLinearDistance wheelDistance : wheelDistances){
      average += wheelDistance.getDistanceMeters();
    }
    average = average/wheelDistances.length;
    return average;

  }

  @Override
  public double leftDistance() {
      // TODO Auto-generated method stub
      return wheelAverage(leftWheels);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dEstimator.update(getRotation(), leftDistance(), rightDistance());
  }
}