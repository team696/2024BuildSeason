package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.TalonFactory;
import frc.robot.util.Log.Debug;
import frc.robot.util.Log.PLog;
import frc.robot.util.Util;

public class Shooter extends SubsystemBase {
    private static Shooter m_Shooter;

    private TalonFactory m_Top;
    private TalonFactory m_Bottom;

    private TalonFactory m_Serializer;

    private TalonFactory m_AngleMotor;

    private DutyCycleEncoder m_Encoder;

    private BangBangController m_shooterController;

    private DutyCycleOut m_PositionRequest;

    private ProfiledPIDController m_AngleTrapPID;
    private ArmFeedforward m_AngleFeedForward;

    private DigitalInput m_BeamBreak;

    public static class State {
        public double angle;
        public double topSpeed;
        public double bottomSpeed;
        public State(double a, double t, double b) {
        angle = a;
        topSpeed = t;
        bottomSpeed = b;
        }
    }

    public static Shooter get() {
        if (m_Shooter == null) {
            m_Shooter = new Shooter();
        }
        return m_Shooter;
    }
  
    public Shooter() {
        m_Top = new TalonFactory(10, Constants.canivoreName, Constants.CONFIGS.shooter_Top, "Shooter Top Roller");
        m_Bottom = new TalonFactory(11, Constants.canivoreName, Constants.CONFIGS.shooter_Bottom, "Shooter Bottom Roller");
        m_AngleMotor = new TalonFactory(12, Constants.canivoreName, Constants.CONFIGS.shooter_Angle, "Shooter Angle");
        m_Serializer = new TalonFactory(13, Constants.canivoreName, Constants.CONFIGS.shooter_Serializer, "Shooter Serializer");

        m_Encoder = new DutyCycleEncoder(7);
        
        m_shooterController = new BangBangController();

        m_PositionRequest = new DutyCycleOut(0);

        m_AngleTrapPID = new ProfiledPIDController(1/36.0, 0, 0, new TrapezoidProfile.Constraints(3000, 2000));
        m_AngleTrapPID.reset(0);
        m_AngleFeedForward = new ArmFeedforward(0.05, 1/56.0, 0, 0);

        m_BeamBreak = new DigitalInput(9);

        m_AngleMotor.setPosition(0);
    }

    /** RPM */
    public void setSpeed(double top, double bottom) { 
        double topSpeed = m_shooterController.calculate(m_Top.getVelocity() * 60.0, top);
        double bottomSpeed = m_shooterController.calculate(m_Bottom.getVelocity() * 60, bottom); 

        if (topRollerVelocity() < 250) {
            topSpeed *= 0.45;
        }

        if (bottomRollerVelocity() < 250) {
            bottomSpeed *= 0.45;
        }

        m_Top.PercentOutput(topSpeed);
        m_Bottom.PercentOutput(bottomSpeed);
    }

    public void setShooterSpeedPercent(double percent) {
        m_Top.PercentOutput(percent);
        m_Bottom.PercentOutput(percent);
    }

    @Debug
    public double getAngle() { //fix this ...maybe not
        if (m_Encoder.isConnected())
            return (360 * (1 - m_Encoder.getAbsolutePosition()) + 180) % 360 - Constants.shooter.AngleOffset;

        return m_AngleMotor.getPosition() * 360;
    }

    /** Percent */
    public void setSerializerSpeedPercent(double output) {
        m_Serializer.PercentOutput(output);
    }

    public void setAngle(double desired) {
        // TODO: Add acceleration to feed forward control if needed
        //double acceleration = (controller.getSetpoint().velocity - lastSpeed) / (Timer.getFPGATimestamp() - lastTime)
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
        if (m_Top.getVelocity() * 60 < top - tolerance) return false;
        if (m_Bottom.getVelocity() * 60 < bottom - tolerance) return false;

        return true;
    }

    public Command Intake() {
        return this.runEnd(()-> { setSerializerSpeedPercent(0.25); setAngle(50); }, ()->{setSerializerSpeedPercent(0); stopAngle();}).onlyWhile(()->getBeamBreak());
    }

    public State getStateFromDist(double dist) {
        dist = Util.clamp(dist, Constants.shooter.distToState.firstKey() + 0.01,Constants.shooter.distToState.lastKey() - 0.01);
        Map.Entry<Double, State> lower = Constants.shooter.distToState.floorEntry(dist);
        Map.Entry<Double, State> higher = Constants.shooter.distToState.ceilingEntry(dist);

        return new State(Util.lerp((dist - lower.getKey())/(higher.getKey() - lower.getKey()), lower.getValue().angle, higher.getValue().angle), higher.getValue().topSpeed, higher.getValue().bottomSpeed);
    }

    public void stopShooter() {
        m_Top.stop();
        m_Bottom.stop();
    }

    public void stopAngle() {
        m_AngleMotor.stop();
    }

    public void stopSerializer() {
        m_Serializer.stop();
    }

    @Debug
    public boolean getBeamBreak() {
        return m_BeamBreak.get();
    }

    public void AnglePercent(double percent) {
        m_AngleMotor.PercentOutput(percent);
    }

    public Command defaultCom() { //TODO: Make this slowly run the shooter rollers as well?
        return this.runEnd(()->setAngle(30), ()->stopAngle());
    }

    @Override
    public void periodic() {
        if (!m_Encoder.isConnected()) {
            PLog.unusual("Shooter", "Encoder Not Found!");
        } else {
           // m_AngleMotor.setPosition(getAngle()/360, 0.00001); // This SLows the fuck out of the code, changing timeout does nothing?
        }
        
    }

    @Debug
    public double topRollerVelocity() {
        return m_Top.getVelocity() * 60;
    }

    @Debug 
    public double bottomRollerVelocity() {
        return m_Bottom.getVelocity() * 60;
    }

    @Debug
    public double angleMotorPosition() {
        return m_AngleMotor.getPosition();
    }


    @Override
    public void initSendable(SendableBuilder builder) {    } 
}
