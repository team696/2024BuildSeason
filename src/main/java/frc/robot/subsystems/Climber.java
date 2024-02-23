package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.TalonFactory;

public class Climber extends SubsystemBase {
  private static Climber m_Climber;

  private TalonFactory m_Master;

  private TalonFactory m_Follower;

  private PositionDutyCycle m_Controller;

  public enum Position {
    down,
    up
  }

  public static Climber get() {
    if (m_Climber == null) {
      m_Climber = new Climber();
    }
    return m_Climber;
  }

  /** Creates a new Climber. */
  private Climber() {
    m_Master = new TalonFactory(15, Constants.canivoreName, Constants.CONFIGS.climber_Master, "Climber Master");
    m_Follower = new TalonFactory(16, Constants.canivoreName, Constants.CONFIGS.climber_Follower, "Climber Follower");

    m_Follower.Follow(m_Master, false);

    m_Controller = (new PositionDutyCycle(0)).withFeedForward(0);
  }

  public void position(Position pos) { 
    switch (pos) {
      case down:
          m_Master.setControl(m_Controller.withPosition(4.889));
      break;
      case up:
          m_Master.setControl(m_Controller.withPosition(0));
      break;
    }
  }

  public void runMotorsPercent(double output) {
    m_Master.PercentOutput(output);
  }

  public Command runClimberPercent(double output) {
    return this.runEnd(()->runMotorsPercent(output),()->runMotorsPercent(0));
  }

  public void stopMotors() {
    m_Master.stop();
  }

  public void stop () {
    m_Master.stop();
  }

  public double getPosition() {
    return m_Master.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position", this::getPosition, null);
  }
}
