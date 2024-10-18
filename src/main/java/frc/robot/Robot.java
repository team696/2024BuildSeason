package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Amp;
import frc.robot.commands.Drop;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.Pass;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterDefault;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Trap;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
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
  
  private void configureBinds() {
    //Swerve.get().setDefaultCommand(new TeleopSwerve(Controls.joystickPanel, 1, 0, 2, Constants.deadBand,true, true));
    //leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
    TeleopSwerve controllerTeleop=new TeleopSwerve(()->1.0, ()->Swerve.get().AngleForSpeaker(), true, false);
    TeleopSwerve.config(()->Controls.joystickPanel.getRawAxis(1), ()->Controls.joystickPanel.getRawAxis(0), ()->Controls.joystickPanel.getRawAxis(2), ()->Controls.rightJoy.getAsBoolean(), Constants.deadBand);
    Controls.leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
  }

    private void configureOperatorBinds() {
        Controls.Shoot.whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));
        //Controls.Source.whileTrue(new ShooterIntake().alongWith(new TeleopSwerve(Controls.joystickPanel, 1, 0, 2, ()->135, Constants.deadBand, true, true)));
        //Controls.Amp.whileTrue(new Amp(Controls.Rollers::getAsBoolean).alongWith(new TeleopSwerve(Controls.joystickPanel, 1, 0, 2, ()->-90, Constants.deadBand, true, true)));
        Controls.Trap.whileTrue(Auto.PathFind(Constants.Field.RED.Trap).andThen(new Trap(()->true)));
        Controls.Drop.whileTrue(new Drop());
        Controls.Gyro.onTrue(new InstantCommand(()->Swerve.get().zeroYaw())); 

        Controls.Rightest.whileTrue(new ManualShoot());
    }

    private void configureControllerBinds() { 


        TeleopSwerve controllerTeleop=new TeleopSwerve(()->1.0, ()->Swerve.get().AngleForSpeaker(), true, false);
        TeleopSwerve.config(()->(-Controls.controller.getRawAxis(0)), ()->(-Controls.controller.getRawAxis(1)), ()->-Controls.controller.getRawAxis(4),Controls.controller.button(10)::getAsBoolean, 0.04);
        //controllerTeleop.setAim(()->Controls.controller.button(10).getAsBoolean());
        Swerve.get().setDefaultCommand(controllerTeleop);

        // B - PASS
        Controls.controller.b().whileTrue(new Pass().alongWith(new TeleopSwerve(()->Swerve.get().getAngleToCorner().rotateBy(Rotation2d.fromDegrees((10)) ))));
        // HOME - ZERO YAW
        Controls.controller.button(9).onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
        // RIGHT BUMPER - SHOOT
        Controls.controller.rightBumper().whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));
        // A - AMP
        Controls.controller.a().whileTrue(new Amp(Controls.controller.rightBumper()::getAsBoolean));
        // LEFT BUMPER - PERFORM SOURCE INTAKE
        Controls.controller.leftBumper().whileTrue(new ShooterIntake());
        Controls.controller.start().onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
    }
    private void configureDualControllerBinds(){
      CommandXboxController driver= new CommandXboxController(0);
      CommandXboxController operator= new CommandXboxController(1);

      driver.x().onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
//      TeleopSwerve controllerTeleop=new TeleopSwerve(()->(-driver.getRawAxis(1)), ()->(-driver.getRawAxis(0)), ()->-driver.getRawAxis(4), ()->false, ()->Swerve.get().AngleForSpeaker().getDegrees(), 0, true, true);
//      controllerTeleop.setAim(driver.button(10)::getAsBoolean);
         TeleopSwerve controllerTeleop=new TeleopSwerve(()->1.0, ()->Swerve.get().AngleForSpeaker().rotateBy(Rotation2d.fromDegrees(4)), true, false);
        TeleopSwerve.config(()->(-driver.getRawAxis(0)), ()->(-driver.getRawAxis(1)), ()->-driver.getRawAxis(4),()->driver.y().getAsBoolean(), 0.03);
        Swerve.get().setDefaultCommand(controllerTeleop);


      // let other buttons do pathfinding


      // Operator Buttons
      // B - PASS
      operator.b().whileTrue(new Pass().alongWith(new TeleopSwerve(()->Swerve.get().getAngleToCorner().rotateBy(Rotation2d.fromDegrees((10))))));
      // X - SHOOT
      operator.rightBumper().whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));
      // A - AMP
      operator.a().whileTrue(new Amp(Controls.controller.rightBumper()::getAsBoolean));
      // Y - PERFORM SOURCE INTAKE
      operator.leftBumper().whileTrue(new ShooterIntake().alongWith(new TeleopSwerve(()->(DriverStation.getAlliance().get()==Alliance.Red?Constants.Field.RED.Source.getRotation():Constants.Field.BLUE.Source.getRotation()))));
      operator.y().whileTrue(new Drop());
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
        configureDualControllerBinds();
        //configureBinds(); 
        //configureOperatorBinds();
        
        SmartDashboard.putData(Swerve.get());
        SmartDashboard.putData(Shooter.get());
        SmartDashboard.putNumber("Shooter Offset", 0);

        Shooter.get().setDefaultCommand(new ShooterDefault());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        //if (Constants.DEBUG) 
            //SmartDashboard.putData(m_PDH);
    /** Used to update network tables faster (causes lag!) */
        //NetworkTableInstance.getDefault().flush();

        lastPeriodTimeDelta = Timer.getFPGATimestamp() - lastPeriodicTime;
        lastPeriodicTime = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledInit() {
    Shooter.get().disable();
    PLog.info("Robot", "Disabled");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
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
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    PLog.info("Robot", "Teleop Init");
  }

  @Override
  public void teleopPeriodic() {
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
