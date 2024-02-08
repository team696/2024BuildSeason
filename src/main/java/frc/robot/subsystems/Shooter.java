package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Log;

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
  private DutyCycleOut m_PositionRequest;

  private PIDController m_AnglePID;

  private DigitalInput m_BeamBreak;

  public double AngleGoal = 0;

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

    m_Encoder = new DutyCycleEncoder(7);
    m_Encoder.setConnectedFrequencyThreshold(2);
    
    m_shooterController = new BangBangController();

    m_TopRequest = new DutyCycleOut(0);
    m_BottomRequest = new DutyCycleOut(0);

    m_PositionRequest = new DutyCycleOut(0);

    m_AnglePID = new PIDController(1/48.0, 0, 0);

    m_BeamBreak = new DigitalInput(9);
  }

  /** RPM */
  public void setSpeed(double top, double bottom) {
   m_Top.setControl(m_TopRequest.withOutput(m_shooterController.calculate(m_Top.getVelocity().getValueAsDouble() * 60.0, top)));
   m_Bottom.setControl(m_BottomRequest.withOutput(m_shooterController.calculate(m_Bottom.getVelocity().getValueAsDouble() * 60, bottom)));
  }

  public void setShooterSpeedPercent(double percent) {
    m_Top.set(percent);
    m_Bottom.set(percent);
  }

  public double getAngle() {
    return -360 * m_Encoder.getAbsolutePosition() - Constants.Shooter.AngleOffset;
  }

  /** Percent */
  public void setSerializerSpeedPercent(double output) {
    m_Serializer.set(output);
  }

  public void setAngle(double desired) {
    m_AngleMotor.setControl(m_PositionRequest.withOutput(m_AnglePID.calculate(getAngle(), desired)));  
  }

  public boolean upToSpeed(double top, double bottom) {
    if (m_Top.getVelocity().getValueAsDouble() * 60 < top - 100) return false;
    if (m_Bottom.getVelocity().getValueAsDouble() * 60 < bottom - 100) return false;

    return true;
  }

  public Command Intake() {
    return this.runEnd(()->setSerializerSpeedPercent(0.25), ()->setSerializerSpeedPercent(0)).onlyWhile(()->getBeamBreak());
  }

  public void stopShooter() {
    m_Top.stopMotor();
    m_Bottom.stopMotor();
  }

  public void stopAngle() {
    m_AngleMotor.stopMotor();
  }

  public boolean getBeamBreak() {
    return m_BeamBreak.get();
  }

  public void AnglePercent(double percent) {
    m_AngleMotor.set(percent);
  }

  public Command MoveAngle(double percent) {
    return Commands.runEnd(()->AnglePercent(percent), ()->AnglePercent(0));
  }

  public Command PositionAngle(double goal) {
    return Commands.runEnd(()->setAngle(goal), ()->AnglePercent(0));
  }

  public Command holdAngle() {
    return this.runEnd(()->setAngle(AngleGoal), ()->stopAngle());
  }

  @Override
  public void periodic() {
   // setAngle(AngleGoal);

   if (!m_Encoder.isConnected()) {
    Log.unusual("Shooter", "Encoder Not Found!");
   }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.DEBUG) {
    	builder.addDoubleProperty("Encoder", ()->getAngle(), null);
    	builder.addBooleanProperty("Beam Break", ()->getBeamBreak(), null);

    	builder.addDoubleProperty("Angle Motor Position", ()->m_AngleMotor.getPosition().getValueAsDouble(), null);

    	builder.addDoubleProperty("Top Velocity", ()->m_Top.getVelocity().getValueAsDouble(), null);
    	builder.addDoubleProperty("Bottom Velocity", ()->m_Bottom.getVelocity().getValueAsDouble(), null);
    }

    builder.addDoubleProperty("Angle", null, (n)->AngleGoal = Math.max(0,Math.min(n, 60)));
    SmartDashboard.putNumber("Shooter/Angle", 0);
  } 
}
