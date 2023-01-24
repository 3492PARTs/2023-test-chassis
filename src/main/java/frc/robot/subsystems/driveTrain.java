// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import PARTSlib2023.PARTS.frc.Utils.Interfaces.beanieDriveTrain;
import PARTSlib2023.PARTS.frc.Utils.sensors.wheelLinearDistance;
import PARTSlib2023.PARTS.frc.commands.joystickDrive;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class driveTrain extends beanieDriveTrain {


  wheelLinearDistance[] leftWheels = new wheelLinearDistance[2];
  wheelLinearDistance[] rightWheels = new wheelLinearDistance[2];
  private final Field2d m_field = new Field2d();

    static CANSparkMax left1 = new CANSparkMax(0, MotorType.kBrushless);
    static CANSparkMax left2 = new CANSparkMax(1, MotorType.kBrushless);

    static CANSparkMax right1 = new CANSparkMax(2, MotorType.kBrushless);
    static CANSparkMax right2 = new CANSparkMax(3, MotorType.kBrushless);

  DifferentialDriveKinematics dKinematics = new DifferentialDriveKinematics((Double) 20.0); //tbd
  DifferentialDrivePoseEstimator dEstimator = new DifferentialDrivePoseEstimator(dKinematics, getRotation(), leftDistance(), rightDistance(), null);
  private static driveTrain mDriveTrain = new driveTrain();
    private static double kv;
    private static double ka;
    private static double ks;
    public  static HashMap<String, Command> eventMap = new HashMap<>();

    

  /** Creates a new driveTrain. */
  public driveTrain() {
    super(new AHRS() , new MotorControllerGroup(left1, left2), new MotorControllerGroup(right1, right2));     
    SmartDashboard.putData(m_field);
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

    public Supplier<Pose2d> getPoseSupplier(){
        Supplier<Pose2d> s = () -> dEstimator.getEstimatedPosition();
        return s;
    }

    public DifferentialDriveKinematics getKinematics() {
        return dKinematics;
    }

    public void resetPose(Pose2d newPose){
        dEstimator.resetPosition(getRotation(), leftDistance(), rightDistance(), newPose);
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

    public BiConsumer<Double, Double> getBiConsumer() {
        BiConsumer<Double, Double> biC = (leftVoltage, rightVoltage) -> {
          leftControllerGroup.setVoltage(leftVoltage);
          rightControllerGroup.setVoltage(rightVoltage);
          super.mDrive.feed();  
        };
        return biC;
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


    public double getLeftVelocity(){
        return -(leftWheels[0].getVelocity() + leftWheels[1].getVelocity())/2;
        }

    public double getRightVelocity(){
        return (rightWheels[0].getVelocity() + rightWheels[1].getVelocity())/2;
        }
    




    
  public Supplier<DifferentialDriveWheelSpeeds> getWheelSpeedSupplier(){
    Supplier<DifferentialDriveWheelSpeeds> s = () -> new DifferentialDriveWheelSpeeds((getLeftVelocity() * 6*Math.PI)/8.01, (getRightVelocity()* 6*Math.PI)/8.01);
    return s;
 }
 public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

     PPRamseteCommand controller1 = new PPRamseteCommand(
        traj,
        driveTrain.getInstance().getPoseSupplier(),
        new RamseteController(),
        new SimpleMotorFeedforward(ks, kv, ka),
        driveTrain.getInstance().getKinematics(),
        driveTrain.getInstance().getWheelSpeedSupplier(),
        new PIDController(1, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(1, 0, 0),
        driveTrain.getInstance().getBiConsumer(),
        this
        );

    return controller1;
 }




}