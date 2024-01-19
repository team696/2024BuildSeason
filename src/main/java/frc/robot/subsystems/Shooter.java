package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Shooter extends SubsystemBase {
  private static Shooter m_Shooter;

  private TalonFX m_Master;
  private TalonFX m_Follower;

  private BangBangController m_shooterController = new BangBangController();

  private DutyCycleOut m_Request = new DutyCycleOut(0);

  public static synchronized Shooter get() {
    if (m_Shooter == null) {
      m_Shooter = new Shooter();
    }
    return m_Shooter;
  }
  
  public Shooter() {
    m_Master = new TalonFX(10);
    m_Follower = new TalonFX(11);

    m_Master.getConfigurator().apply(Constants.CONFIGS.shooterMasterFXConfig);
    m_Follower.getConfigurator().apply(Constants.CONFIGS.shooterFollowerFXConfig);

    m_Follower.setControl(new Follower(m_Master.getDeviceID(), false));
  }

  /** RPM */
  public void setSpeed(double speed) {
    m_Master.setControl(m_Request.withOutput(m_shooterController.calculate(m_Master.getVelocity().getValueAsDouble() * 60.0, speed)));
  }

  public void stop() {
    m_Master.stopMotor();
  }

  @Override
  public void periodic() {
    
  }
}
