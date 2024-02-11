package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            Constants.CONFIGS.FollowConfig,
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

        //NamedCommands.registerCommand("Shoot", new Shoot(3000,2500,1, ()->30));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Dashboard.push("selected_auto", ()->m_instance.autoChooser.getSelected().getName());
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
}
