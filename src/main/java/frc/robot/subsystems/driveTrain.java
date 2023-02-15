// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import PARTSlib2023.PARTS.frc.Utils.Interfaces.beanieDriveTrain;

import PARTSlib2023.PARTS.frc.Utils.logging.Logger;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class driveTrain extends beanieDriveTrain {



 
  private final Field2d sim_field = new Field2d();
  

    // static CANSparkMax left1 = new CANSparkMax(18, MotorType.kBrushless);
    // static CANSparkMax left2 = new CANSparkMax(23, MotorType.kBrushless);

    // static CANSparkMax right1 = new CANSparkMax(24, MotorType.kBrushless);
    // static CANSparkMax right2 = new CANSparkMax(12, MotorType.kBrushless);


    static CANSparkMax left1 = new CANSparkMax(9, MotorType.kBrushless);
    static CANSparkMax left2 = new CANSparkMax(24, MotorType.kBrushless);

    static CANSparkMax right1 = new CANSparkMax(20, MotorType.kBrushless);
    static CANSparkMax right2 = new CANSparkMax(12, MotorType.kBrushless);

    private Field2d m_field = new Field2d();

    SimDouble m_simAngle;

    
    private DifferentialDrivetrainSim m_driveSim = DifferentialDrivetrainSim.createKitbotSim(
      KitbotMotor.kDoubleNEOPerSide, // 2 CIMs per side.
      KitbotGearing.k8p45,        // 10.71:1
      KitbotWheelSize.kSixInch,    // 6" diameter wheels.
      null                         // No measurement noise.
    );




    



  



  DifferentialDriveKinematics dKinematics = new DifferentialDriveKinematics( Units.inchesToMeters(21.12)); //tbd
  DifferentialDrivePoseEstimator dEstimator = new DifferentialDrivePoseEstimator(dKinematics, getRotation(), leftDistance(), rightDistance(), new Pose2d(1.0, 3.0 ,getRotation()));
  private static driveTrain mDriveTrain = new driveTrain();
    private static double kv = 1.055;
    private static double ka = .27947;
    private static double ks = .2432 ;
    public  static HashMap<String, Command> eventMap = new HashMap<>();
    

    

  /** Creates a new driveTrain. */
  public driveTrain() {
    super(new AHRS() , new MotorControllerGroup(left1, left2), new MotorControllerGroup(right1, right2));     
    Shuffleboard.getTab("primary").add(m_field);
    if(Robot.isSimulation()){
      REVPhysicsSim.getInstance().addSparkMax(left1, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(left2, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(right1, DCMotor.getNeo550(1));
      REVPhysicsSim.getInstance().addSparkMax(right2, DCMotor.getNeo550(1));

      var simGYro = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(simGYro, "Yaw")));
      SmartDashboard.putData("Field", m_field);
    }
  }



  
  public static driveTrain getInstance() {
      return mDriveTrain;
  }

  @Override
  public Pose2d currentPose() {
      return dEstimator.getEstimatedPosition();
  }

      /**
     * @return in meters
     */
  @Override
  public double rightDistance() {
      // TODO Auto-generated method stub

      return Units.inchesToMeters((right1.getEncoder().getPosition() * 6 * Math.PI)/ 8.01);
  }


  // private double wheelAverage(wheelLinearDistance[] wheelDistances){

  //   double average = 0;
  //   for(wheelLinearDistance wheelDistance : wheelDistances){
  //     average += wheelDistance.getDistanceMeters();
  //   }
  //   average = average/wheelDistances.length;
  //   return average;

  // }

    /**
     * @return in meters
     */
    @Override
    public double leftDistance() {
        // TODO Auto-generated method stub
        return -Units.inchesToMeters((left1.getEncoder().getPosition() * 6 * Math.PI)/ 8.01);
    }

    public Supplier<Pose2d> getPoseSupplier(){
        Supplier<Pose2d> s = () -> dEstimator.getEstimatedPosition();
        return s;
    }

    /**
     * 
     * @return dKinematics is in meters
     */
    public DifferentialDriveKinematics getKinematics() {
        return dKinematics;
    }

    public void resetPose(Pose2d newPose){
        
        dEstimator.resetPosition(getRotation(), leftDistance(), rightDistance(), newPose);
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        m_field.setRobotPose(dEstimator.getEstimatedPosition());

    }

    @Override
    public void autonomousSetup() {
        // TODO Auto-generated method stub
        
        
        
    }

    public BiConsumer<Double, Double> getBiConsumer() {
        BiConsumer<Double, Double> biC = (leftVoltage, rightVoltage) -> {
          leftControllerGroup.setVoltage(-leftVoltage);
          rightControllerGroup.setVoltage(rightVoltage);
          super.mDrive.feed();  
        };
        return biC;
    }

    @Override
    public void teleopSetup() {
        // TODO Auto-generated method stub
        //driveTrain.getInstance().setDefaultCommand(new joystickDrive(driveTrain.getInstance(), RobotContainer.driverController));
    }


    public void updatePoseVisually(Pair<Pose2d, Double> pose) {

        dEstimator.addVisionMeasurement(pose.getFirst(), pose.getSecond());
        m_field.setRobotPose(dEstimator.getEstimatedPosition());
      

    }


    /**
     * 
     * @return in rotations per second
     */
    public double getLeftVelocity(){
        return -(left1.getEncoder().getVelocity()/60);
        }
        /**
         * 
         * @return in rotations per second
         */
    public double getRightVelocity(){
        return right1.getEncoder().getVelocity()/60;
        }
    




    /**
     * 
     * @return in meters per second
     */
  public Supplier<DifferentialDriveWheelSpeeds> getWheelSpeedSupplier(){
    Supplier<DifferentialDriveWheelSpeeds> s = () -> new DifferentialDriveWheelSpeeds((Units.inchesToMeters(getLeftVelocity() * 6*Math.PI)/8.01), (Units.inchesToMeters(getRightVelocity()* 6*Math.PI)/8.01));
    return s;
 }
 public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

  if(isFirstPath){
    this.resetPose(traj.getInitialPose());
  }

     PPRamseteCommand controller1 = new PPRamseteCommand(
        traj,
        driveTrain.getInstance().getPoseSupplier(),
        new RamseteController(),
        new SimpleMotorFeedforward(ks, kv, ka),
        driveTrain.getInstance().getKinematics(),
        driveTrain.getInstance().getWheelSpeedSupplier(),
        new PIDController(.06, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(.06, 0, 0),
        driveTrain.getInstance().getBiConsumer(),
        this
        );

    return controller1;
 }

 

public void simulationPeriodic() {
  // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.

  // Advance the model by 20 ms. Note that if you are running this
  // subsystem in a separate thread or have changed the nominal timestep
  // of TimedRobot, this value needs to match it.



  if(Robot.isSimulation()){
    REVPhysicsSim.getInstance().run();
  }


  // Update all of our sensors.

}




}