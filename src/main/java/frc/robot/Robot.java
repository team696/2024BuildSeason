package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Amp;
import frc.robot.commands.Drop;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootIntakeAmp;
import frc.robot.commands.ShooterDefault;
import frc.robot.commands.ShooterIntake;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.Trap;
import frc.robot.commands.intake;
import frc.robot.commands.intakeAmp;
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
  
  private void configureBinds() {
    Swerve.get().setDefaultCommand(new TeleopSwerve(Controls.joystickPanel, 1, 0, 2, Constants.deadBand,true, true));
    //leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
  }

    private void configureOperatorBinds() {
        Controls.Shoot.whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));
        Controls.Ground.whileTrue(new intake());
        Controls.Source.whileTrue(new ShooterIntake().alongWith(new TeleopSwerve(Controls.joystickPanel, 1, 0, 2, ()->135, Constants.deadBand, true, true)));
        Controls.Amp.whileTrue(new Amp(Controls.Rollers::getAsBoolean).alongWith(new TeleopSwerve(Controls.joystickPanel, 1, 0, 2, ()->-90, Constants.deadBand, true, true)));
        Controls.Trap.whileTrue(Auto.PathFind(Constants.Field.RED.Trap).andThen(new Trap(()->true)));
        Controls.Drop.whileTrue(new Drop());
        Controls.Gyro.onTrue(new InstantCommand(()->Swerve.get().zeroYaw())); 

        Controls.ExtraC.whileTrue(new intakeAmp());
        Controls.ExtraA.whileTrue(new ShootIntakeAmp(Controls.Rollers::getAsBoolean));
        Controls.Rightest.whileTrue(new ManualShoot());
    }

    private void configureControllerBinds() { 
        Controls.controller.rightBumper().whileTrue(new intake());

        Controls.controller.y().whileTrue(new ShooterIntake());

       // Swerve.get().setDefaultCommand(new TeleopSwerve(()->-controller.getRawAxis(1), ()->-controller.getRawAxis(0), ()->-controller.getRawAxis(4), ()->false, 0.08,true, true));
        Controls.controller.button(8).onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
        Controls.controller.a().whileTrue(new Shoot(()->Swerve.get().DistToSpeaker(), ()->true));

        Controls.controller.x().whileTrue(new Amp(Controls.controller.b()::getAsBoolean));
        Controls.controller.leftBumper().whileTrue(new Trap(()->true));
        Controls.controller.leftTrigger(0.9).whileTrue(Intake.get().goToAngle(Position.down));
        Controls.controller.rightTrigger(0.9).whileTrue(Intake.get().goToAngle(Position.stowed));

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
        SmartDashboard.putData(Shooter.get());
        SmartDashboard.putNumber("Shooter Offset", 0);

        Shooter.get().setDefaultCommand(new ShooterDefault());
        Intake.get().setDefaultCommand(Intake.get().goToAngle(Position.stowed));

        LED.get();

        Logger.registerLoggable(type.debug, "PDH Voltage",m_PDH::getVoltage);
        Logger.registerLoggable(type.debug, "Periodic Time Delta", ()->lastPeriodTimeDelta);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (Constants.DEBUG) 
            SmartDashboard.putData(m_PDH);
    /** Used to update network tables faster (causes lag!) */
        //NetworkTableInstance.getDefault().flush();

        lastPeriodTimeDelta = Timer.getFPGATimestamp() - lastPeriodicTime;
        lastPeriodicTime = Timer.getFPGATimestamp();
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
    Intake.get().setDefaultCommand(Intake.get().goToAngle(Position.down));
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
        Intake.get().setDefaultCommand(Intake.get().goToAngle(Position.stowed));
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
    LED.get().intake = Controls.Climb.getAsBoolean();
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
