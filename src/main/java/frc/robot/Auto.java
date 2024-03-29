package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Log.PLog;
import frc.robot.commands.Rotate;
import frc.robot.commands.Shoot;
import frc.robot.commands.intake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Dashboard;

public class Auto {
    public static Auto m_instance;
    private Swerve m_swerve;

    private final SendableChooser<Command> autoChooser;

    private Auto () {
        m_swerve = Swerve.get();
        
        AutoBuilder.configureHolonomic(
            m_swerve::getPose, 
            m_swerve::resetPose, 
            m_swerve::getRobotRelativeSpeeds,
            m_swerve::Drive, 
            Constants.Auto.FollowConfig,
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            m_swerve
        );

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Constants.Field.sim.getObject("Target").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            Constants.Field.sim.getObject("Path").setPoses(poses);
        });

        NamedCommands.registerCommand("Shoot", new Shoot(()->m_swerve.DistToSpeaker(), true).asProxy()); //.deadlineWith(new suck()).
        NamedCommands.registerCommand("Intake", new intake(true).asProxy());
        NamedCommands.registerCommand("Rotate", new Rotate());
        NamedCommands.registerCommand("Shoot Target", new ParallelCommandGroup(new Shoot(()->m_swerve.DistToSpeaker(), true).asProxy(), new Rotate()));

        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Dashboard.push("selected_auto", ()->m_instance.autoChooser.getSelected().getName());
        autoChooser.onChange((command)-> {
            visualize();
        });
    }

    public static void Initialize(){
        if (m_instance != null) throw new RuntimeException ("Can't Initialize Twice!");

        m_instance = new Auto();
    }

    public static Auto get(){
        if (m_instance == null) throw new RuntimeException ("Please Initialize First!");

        return m_instance;
    }

    public static Command Selected() {
        if (m_instance == null) return new WaitCommand(0);

        return m_instance.autoChooser.getSelected();
    }

    public static Command PathFind(Pose2d end) {
        return AutoBuilder.pathfindToPose(end, new PathConstraints(1, 1, Math.PI,Math.PI));
    }

    public void visualize() {
        List<PathPlannerPath> paths;
        List<Pose2d> pathPoses = new ArrayList<Pose2d>();
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
        } catch (Exception e) {
            PLog.fatalException("Auto", "Failed To Find Path", e);
            return;
        }
        for (int i = 0; i < paths.size(); i++) {
            PathPlannerPath path = paths.get(i);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                path = path.flipPath();
            pathPoses.addAll(path.getPathPoses());
        }
        Constants.Field.sim.getObject("traj").setPoses(pathPoses);
    }
}
