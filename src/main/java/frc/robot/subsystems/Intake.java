package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static Intake m_Intake;

  public static synchronized Intake get() {
    if (m_Intake == null) {
      m_Intake = new Intake();
    }
    return m_Intake;
  }

  /** Creates a new Intake. */
  private Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void initSendable(SendableBuilder builder) {

  }
}
