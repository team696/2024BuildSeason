package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Amp;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Trap;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Util;
import frc.lib.CommandHandler;
import frc.lib.TimedRobot;
import frc.lib.Log.Logger;
import frc.lib.Log.PLog;
import frc.lib.Log.Logger.type;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private PowerDistribution m_PDH;

  private double lastPeriodicTime = 0;
  private double lastPeriodTimeDelta = 0;
  
  private final Joystick joystickPanel = new Joystick(0);
  //private final Joystick operatorPanel = new Joystick(2);

  private final CommandXboxController controller = new CommandXboxController(1);

  private final JoystickButton leftJoy = new JoystickButton(joystickPanel, 1);
  private final JoystickButton rightJoy = new JoystickButton(joystickPanel, 2);

  private void configureBinds() {
    Swerve.get().setDefaultCommand(new TeleopSwerve(joystickPanel, 1, 0, 2, Constants.deadBand,true, true));
    //leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
  }

    private void configureControllerBinds() { 
        controller.y().whileTrue(new ShooterIntake());

        Swerve.get().setDefaultCommand(new TeleopSwerve(()->-controller.getRawAxis(1), ()->-controller.getRawAxis(0), ()->-controller.getRawAxis(4), controller.rightBumper(), 0.08,true, true));
        controller.button(8).onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
        controller.a().whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));
        leftJoy.whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));

        controller.x().whileTrue(new Amp(controller.b()::getAsBoolean));
        controller.leftBumper().whileTrue(new Trap(()->true));
    }

    @Override
    public void robotInit() {
        Util.setRobotType();

        m_PDH = new PowerDistribution(1, ModuleType.kRev);

        DriverStation.silenceJoystickConnectionWarning(true);
    
        LiveWindow.disableAllTelemetry();

        RobotController.setEnabled3V3(false);
        RobotController.setEnabled5V(true);
        RobotController.setEnabled6V(false);

        m_PDH.setSwitchableChannel(true);
        
        //Dashboard.startServer();

        Auto.Initialize();

        configureControllerBinds();   
        //configureBinds(); 

        //SmartDashboard.putData(Swerve.get());

        Shooter.get().setDefaultCommand(Shooter.get().defaultCom());

        Logger.registerLoggable(type.debug, "PDH Voltage",m_PDH::getVoltage);
        Logger.registerLoggable(type.debug, "Periodic Time Delta", ()->lastPeriodTimeDelta);
    }

    @Override
    public void robotPeriodic() {
        lastPeriodTimeDelta = Timer.getFPGATimestamp() - lastPeriodicTime;
        lastPeriodicTime = Timer.getFPGATimestamp();

        CommandScheduler.getInstance().run();
        if (Constants.DEBUG) 
            SmartDashboard.putData(m_PDH);
    /** Used to update network tables faster (causes lag!) */
    //NetworkTableInstance.getDefault().flush();
  }

  @Override
  public void disabledInit() {
    PLog.info("Robot", "Disabled");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.Selected();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    PLog.info("Robot", "Auto Init");
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    PLog.info("Robot", "Teleop Init");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() { }

  @Override
  public void simulationInit() { }

  @Override
  public void simulationPeriodic() { }
}
