package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Climber extends SubsystemBase {
  private static Climber m_Climber;

  private TalonFX m_Master;
  private TalonFX m_Follower;

  private PositionDutyCycle m_Controller;

  public enum Position {
    down,
    up
  }

  public static synchronized Climber get() {
    if (m_Climber == null) {
      m_Climber = new Climber();
    }
    return m_Climber;
  }

  /** Creates a new Climber. */
  private Climber() {
    m_Master = new TalonFX(15);
    m_Follower = new TalonFX(16);

    m_Master.getConfigurator().apply(Constants.CONFIGS.climber_Master);
    m_Follower.getConfigurator().apply(Constants.CONFIGS.climber_Follower);

    m_Follower.setControl(new Follower(m_Master.getDeviceID(), true));

    m_Controller = (new PositionDutyCycle(0)).withFeedForward(0);

    m_Master.setPosition(0);
    m_Follower.setPosition(0);
  }

  public void position(Position pos) {
    switch (pos) {
      case down:
          m_Master.setControl(m_Controller.withPosition(1));
      break;
      case up:
          m_Master.setControl(m_Controller.withPosition(0));
      break;
    }
  }

  public void stop () {
    m_Master.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void initSendable(SendableBuilder builder) {

  }
}
