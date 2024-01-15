package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private PowerDistribution m_PDH;
  
  private final Joystick joystickPanel = new Joystick(0);
  //private final Joystick operatorPanel = new Joystick(2);

  private final JoystickButton leftJoy = new JoystickButton(joystickPanel, 1);
  //private final JoystickButton rightJoy = new JoystickButton(joystickPanel, 2);

  private void configureBinds() {
    Swerve.get().setDefaultCommand(new TeleopSwerve(joystickPanel, 1, 0, 2, false, true));
    leftJoy.onTrue(new InstantCommand(()->Swerve.get().zeroYaw()));
  }

  @Override
  public void robotInit() {
    m_PDH = new PowerDistribution(1, ModuleType.kRev);

    DriverStation.silenceJoystickConnectionWarning(true);
    LiveWindow.disableAllTelemetry();

    Auto.Initialize();

    configureBinds();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(m_PDH);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = Auto.get().Selected();

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
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
