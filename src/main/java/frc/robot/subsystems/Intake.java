package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Intake extends SubsystemBase {
  private static Intake m_Intake;

  private TalonFX m_Linear;
  private TalonFX m_Angle;
  private TalonFX m_Rollers;
  private TalonFX m_SecondAngle;

  private DigitalInput m_BeamBreak;

  public static synchronized Intake get() {
    if (m_Intake == null) {
      m_Intake = new Intake();
    }
    return m_Intake;
  }

  /** Creates a new Intake. */
  private Intake() {
    m_Linear = new TalonFX(18);
    m_Angle = new TalonFX(19);
    m_Rollers = new TalonFX(20);
    m_SecondAngle = new TalonFX(21);

    m_Linear.getConfigurator().apply(Constants.CONFIGS.intake_Linear);
    m_Angle.getConfigurator().apply(Constants.CONFIGS.intake_Angle);
    m_Rollers.getConfigurator().apply(Constants.CONFIGS.intake_Rollers);
    m_SecondAngle.getConfigurator().apply(Constants.CONFIGS.intake_SecondAngle);

    m_Linear.setPosition(0);
    m_Angle.setPosition(0);
    m_SecondAngle.setPosition(0);

    m_BeamBreak = new DigitalInput(9);
  }

  public boolean getBeamBreak() {
    return m_BeamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Beam Break", ()->getBeamBreak(), null);
  }
}
