package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class Intake extends SubsystemBase {
    private static Intake m_Intake;

    private TalonFX m_Linear;
    private TalonFX m_Angle;
    private TalonFX m_Rollers;
    private TalonFX m_SecondAngle;

    public enum Position {
        stowed,
        down, 
        pass,
        amp,
        trap
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
        m_Linear = new TalonFX(18, Constants.canivoreName);
        m_Angle = new TalonFX(19, Constants.canivoreName);
        m_Rollers = new TalonFX(20, Constants.canivoreName);
        m_SecondAngle = new TalonFX(21, Constants.canivoreName);

        m_Linear.getConfigurator().apply(Constants.CONFIGS.intake_Linear);
        m_Angle.getConfigurator().apply(Constants.CONFIGS.intake_Angle);
        m_Rollers.getConfigurator().apply(Constants.CONFIGS.intake_Rollers);
        m_SecondAngle.getConfigurator().apply(Constants.CONFIGS.intake_SecondAngle);

        m_Linear.setPosition(0);
        m_Angle.setPosition(0);
        m_SecondAngle.setPosition(0);
    }

    public void setRollersOutput(double percent) {
        m_Rollers.set(percent);
    }

    public void setLinearOutput(double percent) {
        m_Linear.set(percent);
    }

    public Command runLinear(double percent) {
        return this.runEnd(()->setLinearOutput(percent),()->setLinearOutput(0));
    }

    public void stopRollers() {
        m_Rollers.stopMotor();
    }

    public Command runRollers(double percent) {
        return this.runEnd(()->setRollersOutput(percent),()->setRollersOutput(0));
    }

     public void setAngleOutput(double percent) {
        m_Angle.set(percent);
    }

    public void stopAngle() {
        m_Angle.stopMotor();
    }

    public Command runAngle(double percent) {
        return this.runEnd(()->setAngleOutput(percent),()->stopAngle());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double linearPosition() {
        return m_Linear.getPosition().getValueAsDouble();
    }

    public double mainAngle() {
        return m_Angle.getPosition().getValueAsDouble();
    }

    public double wristAngle() {
        return m_SecondAngle.getPosition().getValueAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Height", this::linearPosition, null);
        builder.addDoubleProperty("Main Angle", this::mainAngle, null);
        builder.addDoubleProperty("Wrist Angle", this::wristAngle, null);
    }
}
