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
import frc.robot.commands.Drop;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Trap;
import frc.robot.commands.intake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.Position;
import frc.robot.util.Constants;
import frc.robot.util.Util;
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
  private final Joystick operatorPanel = new Joystick(1);
  private final Joystick operatorPanelB = new Joystick(2);
  //private final Joystick operatorPanel = new Joystick(2);

  private final CommandXboxController controller = new CommandXboxController(5);

  private final JoystickButton leftJoy = new JoystickButton(joystickPanel, 1);
  private final JoystickButton rightJoy = new JoystickButton(joystickPanel, 2);

    private final JoystickButton Shoot = new JoystickButton(operatorPanel, 1);
    private final JoystickButton Amp = new JoystickButton(operatorPanel, 2);
    private final JoystickButton ExtraA = new JoystickButton(operatorPanel, 3);
    private final JoystickButton Trap = new JoystickButton(operatorPanel, 4);
    private final JoystickButton ExtraB = new JoystickButton(operatorPanel, 6);
    private final JoystickButton Ground = new JoystickButton(operatorPanel, 7);
    private final JoystickButton Source = new JoystickButton(operatorPanel, 8);
    private final JoystickButton Rollers = new JoystickButton(operatorPanel, 9);
    private final JoystickButton Drop = new JoystickButton(operatorPanel, 10);
    private final JoystickButton ExtraC = new JoystickButton(operatorPanel,11);

    private final JoystickButton Climb = new JoystickButton(operatorPanelB, 2);
    private final JoystickButton OhShit = new JoystickButton(operatorPanelB,   3);
    private final JoystickButton Rightest = new JoystickButton(operatorPanelB, 4);
    private final JoystickButton Right = new JoystickButton(operatorPanelB, 5);
    private final JoystickButton Left = new JoystickButton(operatorPanelB, 6);
    private final JoystickButton Gyro = new JoystickButton(operatorPanelB, 7);
    private final JoystickButton Leftest = new JoystickButton(operatorPanelB, 8);

  private void configureBinds() {
    Swerve.get().setDefaultCommand(new TeleopSwerve(joystickPanel, 1, 0, 2, Constants.deadBand,true, true));
    //leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
  }

    private void configureOperatorBinds() {
        Shoot.whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));
        Ground.whileTrue(new intake());
        Source.whileTrue(new ShooterIntake().alongWith(new TeleopSwerve(joystickPanel, 1, 0, 2, ()->135, Constants.deadBand, true, true)));
        Amp.whileTrue(new Amp(Rollers::getAsBoolean).alongWith(new TeleopSwerve(joystickPanel, 1, 0, 2, ()->-90, Constants.deadBand, true, true)));
        Trap.whileTrue(Auto.PathFind(Constants.Field.BLUE.Trap).andThen(new Trap(()->true)));
        Drop.whileTrue(new Drop());
        Gyro.onTrue(new InstantCommand(()->Swerve.get().zeroYaw())); 
    }

    private void configureControllerBinds() { 
        controller.rightBumper().whileTrue(new intake());

        controller.y().whileTrue(new ShooterIntake());

       // Swerve.get().setDefaultCommand(new TeleopSwerve(()->-controller.getRawAxis(1), ()->-controller.getRawAxis(0), ()->-controller.getRawAxis(4), ()->false, 0.08,true, true));
        controller.button(8).onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
        controller.a().whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));
        leftJoy.whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));

        controller.x().whileTrue(new Amp(controller.b()::getAsBoolean));
        controller.leftBumper().whileTrue(new Trap(()->true));
        controller.leftTrigger(0.9).whileTrue(Intake.get().goToAngle(Position.down));
        controller.rightTrigger(0.9).whileTrue(Intake.get().goToAngle(Position.stowed));

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

        //configureControllerBinds();   
        configureBinds(); 
        configureOperatorBinds();
        
        SmartDashboard.putData(Swerve.get());
        SmartDashboard.putData(Intake.get());

        Shooter.get().setDefaultCommand(Shooter.get().defaultCom());
        Intake.get().setDefaultCommand(Intake.get().goToAngle(Position.stowed));

        LED.get();

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
    Intake.get().disable();
    Shooter.get().disable();
    PLog.info("Robot", "Disabled");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    Intake.get().enable();
    Shooter.get().enable();
  }

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
  public void teleopPeriodic() {
    LED.get().intake = Climb.getAsBoolean();
  }

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
