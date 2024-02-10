package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import edu.wpi.first.wpilibj.GenericHID;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TeleopSwerve extends Command {

    private boolean fieldRelative;
    private boolean openLoop;
    private DoubleSupplier translation;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private double deadband;


    private double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    PIDController pidController = new PIDController(0.016  , 0.00, 0);

    private BooleanSupplier rightJoy;
    /**
     * Driver control
     */
    public TeleopSwerve(GenericHID controller, int translationAxis, int strafeAxis, int rotationAxis, double deadband,boolean fieldRelative, boolean openLoop) {
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        translation = ()->-controller.getRawAxis(translationAxis);
        strafe = ()->controller.getRawAxis(strafeAxis);
        rotation = ()->controller.getRawAxis(rotationAxis);

        this.deadband = deadband;

        pidController.setTolerance(1);
        pidController.enableContinuousInput(-180, 180);
        rightJoy = (new JoystickButton(controller, 2));

        addRequirements(Swerve.get());
    }

    public TeleopSwerve(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, BooleanSupplier rightJoy, double deadband ,boolean fieldRelative, boolean openLoop) {
        translation = x;
        strafe = y;
        rotation = r;

        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        this.deadband = deadband; 

        this.rightJoy = rightJoy;
        
        addRequirements(Swerve.get());
    }

    @Override
    public void execute() {
        double yAxis = translation.getAsDouble();
        double xAxis = strafe.getAsDouble();
        double rAxis = rotation.getAsDouble();
        if (rightJoy != null && rightJoy.getAsBoolean()){
            rAxis = pidController.calculate(Swerve.get().getYaw().getDegrees(), Swerve.get().getAngleForSpeaker(Swerve.get().getPose()).getDegrees() - 180);
        } else {
            if (Math.abs(rAxis) > deadband) {
                if (rAxis > 0)
                    rAxis = mapdouble(rAxis, deadband, 1, 0, 1);
                else 
                    rAxis = mapdouble(rAxis, -deadband, -1, 0, -1);
            } else {
                rAxis = 0;
            }
        }
        Rotation2d theta = new Rotation2d(yAxis, xAxis);
        double magnitude = Math.min(Math.sqrt((xAxis * xAxis) + (yAxis * yAxis)),1);
        if (magnitude < deadband) magnitude = 0;
        Translation2d translation = new Translation2d(Math.pow(magnitude, 2), theta).times(Constants.Swerve.maxSpeed);
        double rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        Swerve.get().Drive(translation, rotation, fieldRelative, openLoop);
    }
}
