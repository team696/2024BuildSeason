package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Log;
import frc.robot.util.Util;
import frc.robot.util.Constants.Shooter.State;

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
    private ProfiledPIDController m_AngleTrapPID;
    private ArmFeedforward m_AngleFeedForward;

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
        m_AngleTrapPID = new ProfiledPIDController(1/48.0, 0, 0, new TrapezoidProfile.Constraints(10, 30));
        m_AngleTrapPID.reset(getAngle());
        m_AngleFeedForward = new ArmFeedforward(0, 0, 0, 0);

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
        // TODO: Add acceleration to feed forward control if needed
        //double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime)

        //m_AngleMotor.setControl(m_PositionRequest.withOutput(m_AnglePID.calculate(getAngle(), desired)));  
        double motorSpeed = m_AngleTrapPID.calculate(getAngle(), desired);
        TrapezoidProfile.State desiredState = m_AngleTrapPID.getSetpoint();
        motorSpeed += m_AngleFeedForward.calculate(Units.degreesToRadians(desiredState.position), Units.degreesToRadians(desiredState.velocity));
        m_AngleMotor.setControl(m_PositionRequest.withOutput(motorSpeed));
    }


  /** desired angle, tolerance all in degrees */
  public boolean atAngle(double angle, double tolerance) {
    if (Math.abs(getAngle() - angle) > tolerance) return false;

    return true;
  }

  /** desired top speed, desired bottom speed, tolerance all in rpm */
  public boolean upToSpeed(double top, double bottom, double tolerance) {
    if (m_Top.getVelocity().getValueAsDouble() * 60 < top - tolerance) return false;
    if (m_Bottom.getVelocity().getValueAsDouble() * 60 < bottom - tolerance) return false;

    return true;
  }

  public Command Intake() {
    return this.runEnd(()->setSerializerSpeedPercent(0.25), ()->setSerializerSpeedPercent(0)).onlyWhile(()->getBeamBreak());
  }

  public State getStateFromDist(double dist) {
        if (dist < Constants.Shooter.distToState.firstKey()) {
            dist = Constants.Shooter.distToState.firstKey();
        }
        if (dist > Constants.Shooter.distToState.lastKey())  {
            dist = Constants.Shooter.distToState.lastKey();
        }

        Map.Entry<Double, State> lower = Constants.Shooter.distToState.floorEntry(dist);
        Map.Entry<Double, State> higher = Constants.Shooter.distToState.ceilingEntry(dist);

        return new State(Util.lerp((dist - lower.getKey())/(higher.getKey() - lower.getKey()), lower.getValue().angle, higher.getValue().angle), lower.getValue().topSpeed, lower.getValue().bottomSpeed);
  }

  public void stopShooter() {
    m_Top.stopMotor();
    m_Bottom.stopMotor();
  }

  public void stopAngle() {
    m_AngleMotor.stopMotor();
  }

  public void stopSerializer() {
    m_Serializer.stopMotor();
  }

  public boolean getBeamBreak() {
    return m_BeamBreak.get();
  }

  public void AnglePercent(double percent) {
    m_AngleMotor.set(percent);
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
    	builder.addDoubleProperty("Encoder", this::getAngle, null);
    	builder.addBooleanProperty("Beam Break", this::getBeamBreak, null);

    	builder.addDoubleProperty("Angle Motor Position", ()->m_AngleMotor.getPosition().getValueAsDouble(), null);

    	builder.addDoubleProperty("Top Velocity", ()->m_Top.getVelocity().getValueAsDouble(), null);
    	builder.addDoubleProperty("Bottom Velocity", ()->m_Bottom.getVelocity().getValueAsDouble(), null);
    }
  } 
}
