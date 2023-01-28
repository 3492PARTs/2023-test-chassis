// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import PARTSlib2023.PARTS.frc.Utils.Interfaces.SparkMaxDistanceValue;
import PARTSlib2023.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import PARTSlib2023.PARTS.frc.Utils.sensors.wheelLinearDistance;
import PARTSlib2023.PARTS.frc.commands.joystickDrive;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class driveTrain extends beanieDriveTrain {



  private final Field2d m_field = new Field2d();

    static CANSparkMax left1 = new CANSparkMax(10, MotorType.kBrushless);
    static CANSparkMax left2 = new CANSparkMax(20, MotorType.kBrushless);

    static CANSparkMax right1 = new CANSparkMax(7, MotorType.kBrushless);
    static CANSparkMax right2 = new CANSparkMax(9, MotorType.kBrushless);

    wheelLinearDistance leftWheels[] = new wheelLinearDistance[]{new wheelLinearDistance(8.01, 6*Math.PI, new SparkMaxDistanceValue(left1)),
      new wheelLinearDistance(8.01, 6*Math.PI, new SparkMaxDistanceValue(left2))};
    wheelLinearDistance rightWheels[] = new wheelLinearDistance[]{new wheelLinearDistance(8.01, 6*Math.PI, new SparkMaxDistanceValue(right1)),
       new wheelLinearDistance(8.01, 6*Math.PI, new SparkMaxDistanceValue(right2)) };
  



  DifferentialDriveKinematics dKinematics = new DifferentialDriveKinematics( 20.0); //tbd
  DifferentialDrivePoseEstimator dEstimator = new DifferentialDrivePoseEstimator(dKinematics, getRotation(), leftDistance(), rightDistance(), new Pose2d(0.0, 0.0 ,getRotation()));
  private static driveTrain mDriveTrain = new driveTrain();
    

  /** Creates a new driveTrain. */
  public driveTrain() {
    super(new AHRS() , new MotorControllerGroup(left1, left2), new MotorControllerGroup(right1, right2));     
    Shuffleboard.getTab("primary").add(m_field);
  }

  
  public static driveTrain getInstance() {
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

    @Override
    public void autonomousSetup() {
        // TODO Auto-generated method stub
        
        
    }

    @Override
    public void teleopSetup() {
        // TODO Auto-generated method stub
        this.setDefaultCommand(new joystickDrive(mDriveTrain, RobotContainer.driverController));
    }


    public void updatePoseVisually(Pair<Pose2d, Double> pose) {

        dEstimator.addVisionMeasurement(pose.getFirst(), pose.getSecond());
        m_field.setRobotPose(dEstimator.getEstimatedPosition());
      

    }



}