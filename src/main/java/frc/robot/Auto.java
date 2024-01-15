package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Exmaple;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;

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
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            m_swerve
        );

        NamedCommands.registerCommand("Example Command", new Exmaple());
    
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static void Initialize(){
        if (m_instance != null) throw new RuntimeException ("Can't Initialize Twice!");

        m_instance = new Auto();
    }

    public static Auto get(){
        if (m_instance == null) throw new RuntimeException ("Please Initialize First!");

        return m_instance;
    }

    public Command Selected() {
        return autoChooser.getSelected();
    }
}
