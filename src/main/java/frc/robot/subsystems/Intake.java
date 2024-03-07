package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.SubsystemHandler;
import frc.lib.TalonFactory;
import frc.robot.util.Constants;

public class Intake extends SubsystemHandler {
    private static Intake m_Intake;

    private TalonFactory m_Angle;
    private TalonFactory m_Rollers;
    private TalonFactory m_Serializer;

    private MotionMagicDutyCycle m_AngleControl;
    private ArmFeedforward m_ArmFeedforward;

    private DigitalInput m_BeamBreak;
                              
    public enum Position { 
        stowed(0), 
        down(19.6), 
        passback(14),
        spit(4),
        amp(3);

        public double pos;
        Position(double p) {
            pos = p;
        }
    } 

    public class State {
        double height = 0;
        double angle = 0;
        double tipAngle = 0;
        double rollerSpeed = 0;
        public State(double height, double angle, double tipAngle, double rollerSpeed) {
            this.height = height;
            this.angle = angle;
            this.tipAngle = tipAngle;
            this.rollerSpeed = rollerSpeed;
        }
        public State() {}

    }

    public static Intake get() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }

    /** Creates a new Intake. */
    private Intake() {
        m_Angle = new TalonFactory(18, Constants.canivoreName, Constants.CONFIGS.intake_Angle, "Intake Angle");

        m_Rollers = new TalonFactory(20, Constants.canivoreName, Constants.CONFIGS.intake_Rollers, "Intake Rollers");

        m_Serializer = new TalonFactory(19, Constants.canivoreName, Constants.CONFIGS.intake_Serializer, "Intake Serializer");

        m_Angle.setPosition(0);

        m_AngleControl = new MotionMagicDutyCycle(0);
        m_ArmFeedforward = new ArmFeedforward(0, 0.07, 0);

        m_BeamBreak = new DigitalInput(6);
    }

    public void setRollersOutput(double percent) {
        m_Rollers.PercentOutput(percent);
    }

    public void stopRollers() {
        m_Rollers.stop();
    }

    public Command runRollers(double percent) {
        return this.runEnd(()->setRollersOutput(percent),()->setRollersOutput(0));
    }

     public void setAngleOutput(double percent) {
        m_Angle.PercentOutput(percent);
    }

    public void positionAngle(Position p) {
        m_AngleControl.FeedForward = -1 * m_ArmFeedforward.calculate((12.6 - m_Angle.getPosition()) / 53 * 2 * Math.PI, 0);
        m_Angle.setControl(m_AngleControl.withPosition(p.pos));
    }

    public void goalAngle(double goal) {
        m_Angle.setControl(m_AngleControl.withPosition(goal));
    }

    public Command goToAngle(Position p) {
        return this.runEnd(()->positionAngle(p), ()->stopAngle());
    }

    public void setSerializerSpeedPercent(double percent) {
        m_Serializer.PercentOutput(percent);
    }

    public void stopSerializer() {
        m_Serializer.stop();
    }

    public void stopAngle() {
        m_Angle.stop();
    }

    public Command runAngle(double percent) {
        return this.runEnd(()->setAngleOutput(percent),()->stopAngle());
    }

    public void enable(){
        m_Angle.get().setNeutralMode(NeutralModeValue.Brake);

    }

    public void disable() {
        m_Angle.get().setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double mainAngle() {
        return m_Angle.getPosition();
    }

    public boolean getBeamBreak() {
        return m_BeamBreak.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Main Angle", this::mainAngle, null);
        builder.addBooleanProperty("Beam Break", this::getBeamBreak, null);
    }
}
