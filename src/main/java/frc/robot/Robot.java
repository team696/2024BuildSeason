package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Dashboard;
import frc.robot.util.Log.Logger;
import frc.robot.util.Log.Logger.type;
import frc.robot.util.Util;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private PowerDistribution m_PDH;
  
  private final Joystick joystickPanel = new Joystick(0);
  //private final Joystick operatorPanel = new Joystick(2);

  private final CommandXboxController controller = new CommandXboxController(1);

  private final JoystickButton leftJoy = new JoystickButton(joystickPanel, 1);
  //private final JoystickButton rightJoy = new JoystickButton(joystickPanel, 2);

  private void configureBinds() {
    Swerve.get().setDefaultCommand(new TeleopSwerve(joystickPanel, 1, 0, 2, Constants.deadBand,true, true));
    leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
  }

    private void configureControllerBinds() { 
        //controller.b().whileTrue(Shooter.get().Intake());
        //controller.y().whileTrue(new ShooterIntake());
        controller.button(8).whileTrue(Intake.get().runRollers(0.75));
        //controller.b().whileTrue(Intake.get().runAngle(0.2));
        //controller.y().whileTrue(Intake.get().runAngle(-0.2));

        Swerve.get().setDefaultCommand(new TeleopSwerve(()->-controller.getRawAxis(1), ()->-controller.getRawAxis(0), ()->-controller.getRawAxis(4), controller.rightBumper(), 0.08,true, true));
        //controller.button(8).onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
        //controller.a().whileTrue(new Shoot(()->Swerve.get().DistToSpeaker()));
        controller.b().whileTrue(Intake.get().runLinear(-0.3));
        controller.y().whileTrue(Intake.get().runLinear(0.3));
        
        controller.a().whileTrue(Climber.get().runClimberPercent(0.5));
        controller.x().whileTrue(Climber.get().runClimberPercent(-0.5));

        //OLD METHODS -> UPDATE?
        //controller.a().whileTrue(new Shoot(3500, 3000, 1, ()-> Util.clamp(Util.lerp( Swerve.get().getDistToSpeaker(Swerve.get().getPose()) / 3.3, 60, 28.0),30.0,60.0))); SPEAKER
        //controller.leftBumper().whileTrue(new Shoot(2000,2000,1, ()->60)); TRAP
        //controller.x().whileTrue(new Shoot(550,550,1, ()->57)); AMP
    }

  @Override
  public void robotInit() {
    Logger.init(Logger.type.debug, Shooter.get(), Swerve.get(), Intake.get(), Climber.get()).start();

    Util.setRobotType();

    m_PDH = new PowerDistribution(1, ModuleType.kRev);

    DriverStation.silenceJoystickConnectionWarning(true);
    
    LiveWindow.disableAllTelemetry();

    RobotController.setEnabled3V3(false);
    RobotController.setEnabled5V(true);
    RobotController.setEnabled6V(false);

    m_PDH.setSwitchableChannel(true);

    Dashboard.startServer();

    Auto.Initialize();

    configureBinds();
    configureControllerBinds();

    //Shooter.get().setDefaultCommand(Shooter.get().holdAngle());

    SmartDashboard.putData(Swerve.get());
    SmartDashboard.putData(Shooter.get());
    SmartDashboard.putData(Intake.get());
    SmartDashboard.putData(Climber.get());

    Logger.registerLoggable(type.debug, "PDH Voltage",m_PDH::getVoltage);
  }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (Constants.DEBUG) 
            SmartDashboard.putData(m_PDH);
    /** Used to update network tables faster (causes lag!) */
    //NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.Selected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }


}
