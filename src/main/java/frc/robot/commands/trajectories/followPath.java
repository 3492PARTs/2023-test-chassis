
package frc.robot.commands.trajectories;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.driveTrain;

public class followPath {

        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    
    //ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("new Path", new PathConstraints(1, 1));
    static PathPlannerTrajectory traj = PathPlanner.loadPath("New Path", new PathConstraints(1, 1));
    private static double kv = 1.055;
    private static double ka = .27947;
    private static double ks = .2432 ;
    static Subsystem m_dTrain = driveTrain.getInstance();
    private static Map<String, Command> eventMap;

    
    static RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
        driveTrain.getInstance().getPoseSupplier(), // Pose2d supplier
        driveTrain.getInstance()::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        new RamseteController(), // SwerveDriveKinematics
        driveTrain.getInstance().getKinematics(),
        new SimpleMotorFeedforward(ks, kv, ka),
        driveTrain.getInstance().getWheelSpeedSupplier(),
        new PIDConstants(.01,0,0),
        driveTrain.getInstance().getBiConsumer(),
        eventMap,
        driveTrain.getInstance() // The drive subsystem. Used to properly set the requirements of path following commands
    );

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.



    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.

    //Command fullAuto = autoBuilder.fullAuto(pathGroup);
    
    public static Command getAutoCommand(){
        
        return driveTrain.getInstance().followTrajectoryCommand(traj, false);
    }
}