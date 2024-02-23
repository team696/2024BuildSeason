package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.TalonFactory;

public class Intake extends SubsystemBase {
    private static Intake m_Intake;

    private TalonFactory m_Linear;
    private TalonFactory m_Angle;
    private TalonFactory m_Rollers;
    private TalonFactory m_SecondAngle;
                              
    public enum Position { //LINEAR TOP IS 3.6
        stowed,
        down, // Linear 0.7569, Main Angle 9.232
        pass, //L 1.8, A 7
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
        m_Linear = new TalonFactory(19, Constants.canivoreName, Constants.CONFIGS.intake_Linear, "Intake Linear");
        m_Angle = new TalonFactory(18, Constants.canivoreName, Constants.CONFIGS.intake_Angle, "Intake Angle");
        m_Rollers = new TalonFactory(20, Constants.canivoreName, Constants.CONFIGS.intake_Rollers, "Intake Rollers");
        //m_SecondAngle = new TalonFactory(21, Constants.canivoreName, Constants.CONFIGS.intake_SecondAngle, "Intake Second Angle");

        m_Linear.setPosition(0);
        m_Angle.setPosition(0);
        //m_SecondAngle.setPosition(0);
    }

    public void setRollersOutput(double percent) {
        m_Rollers.PercentOutput(percent);
    }

    public void setLinearOutput(double percent) {
        m_Linear.PercentOutput(percent);
    }

    public Command runLinear(double percent) {
        return this.runEnd(()->setLinearOutput(percent),()->setLinearOutput(0));
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

    public void stopAngle() {
        m_Angle.stop();
    }

    public Command runAngle(double percent) {
        return this.runEnd(()->setAngleOutput(percent),()->stopAngle());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double linearPosition() {
        return m_Linear.getPosition();
    }

    public double mainAngle() {
        return m_Angle.getPosition();
    }

    public double wristAngle() {
        return m_SecondAngle.getPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Height", this::linearPosition, null);
        builder.addDoubleProperty("Main Angle", this::mainAngle, null);
        //builder.addDoubleProperty("Wrist Angle", this::wristAngle, null);
    }
}
