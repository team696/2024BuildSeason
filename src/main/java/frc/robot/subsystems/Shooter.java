package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Shooter extends SubsystemBase {
  private static Shooter m_Shooter;

  private TalonFX m_Top;
  private TalonFX m_Bottom;

  private TalonFX m_Serializer;

  private TalonFX m_AngleMotor;

  private DutyCycleEncoder m_Encoder;

  private BangBangController m_shooterController;

  private DutyCycleOut m_TopRequest;
  private DutyCycleOut m_BottomRequest;
  private PositionDutyCycle m_PositionRequest;

  private PIDController m_AnglePID;

  private final double m_AngleOffset = 0;

  public static synchronized Shooter get() {
    if (m_Shooter == null) {
      m_Shooter = new Shooter();
    }
    return m_Shooter;
  }
  
  public Shooter() {
    m_Top = new TalonFX(10, Constants.canivoreName);
    m_Bottom = new TalonFX(11, Constants.canivoreName);
    m_AngleMotor = new TalonFX(12, Constants.canivoreName);
    m_Serializer = new TalonFX(13, Constants.canivoreName);

    m_Top.getConfigurator().apply(Constants.CONFIGS.shooter_Top);
    m_Bottom.getConfigurator().apply(Constants.CONFIGS.shooter_Bottom);
    m_AngleMotor.getConfigurator().apply(Constants.CONFIGS.shooter_Angle);
    m_Serializer.getConfigurator().apply(Constants.CONFIGS.shooter_Serializer);

    m_Encoder = new DutyCycleEncoder(0);

    m_shooterController = new BangBangController();

    m_TopRequest = new DutyCycleOut(0);
    m_BottomRequest = new DutyCycleOut(0);

    m_PositionRequest = new PositionDutyCycle(0);

    m_AnglePID = new PIDController(1, 0, 0);
  }

  /** RPM */
  public void setSpeed(double top, double bottom) {
   m_Top.setControl(m_TopRequest.withOutput(m_shooterController.calculate(m_Top.getVelocity().getValueAsDouble() * 60.0, top)));
   m_Bottom.setControl(m_BottomRequest.withOutput(m_shooterController.calculate(m_Bottom.getVelocity().getValueAsDouble() * 60, bottom)));
  }

  /** Percent */
  public void setSerializerSpeedPercent(double output) {
    m_Serializer.set(output);
  }

  public void setAngle(double desired) {
    m_AngleMotor.setControl(m_PositionRequest.withPosition(m_AnglePID.calculate(m_Encoder.getAbsolutePosition() * 360 + m_AngleOffset, desired)));  
  }

  public boolean upToSpeed(double top, double bottom) {
    if (m_Top.getVelocity().getValueAsDouble() * 60 > top) return false;
    if (m_Bottom.getVelocity().getValueAsDouble() * 60 < bottom) return false;

    return true;
   }

  //public Command setShooterSpeed(double top, double bottom) {
  //  return Commands.runEnd(()->setSpeed(top, bottom), ()->stopShooter());
 // }

  //public Command setSerializerSpeed(double speed) {
  //  return Commands.runEnd(()->setSerializerSpeedPercent(speed), ()->setSerializerSpeedPercent(0));
  //}

  public Command Shoot(double top, double bottom, double in) {
    return Commands.run(()->setSpeed(top, bottom)).alongWith(Commands.run(()->setSerializerSpeedPercent(in)).onlyIf(()->upToSpeed(top, bottom)));
  }

  public void stopShooter() {
    m_Top.stopMotor();
    m_Bottom.stopMotor();
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Encoder", ()->m_Encoder.getAbsolutePosition() * 360, null);
  }
}
