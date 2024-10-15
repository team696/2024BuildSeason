package frc.robot.commands;

import frc.robot.Controls;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.GenericHID;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {

    private boolean fieldRelative;
    private boolean openLoop;
    private DoubleSupplier translation;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private double deadband;

    PIDController pidController;

    private BooleanSupplier rightJoy;

    private DoubleSupplier goalRotation;
    /**
     * Driver control
     */
    public TeleopSwerve(GenericHID controller, int translationAxis, int strafeAxis, int rotationAxis, DoubleSupplier goal, double deadband,boolean fieldRelative, boolean openLoop) {
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        translation = ()->-controller.getRawAxis(translationAxis);
        strafe = ()->controller.getRawAxis(strafeAxis);
        rotation = ()->-controller.getRawAxis(rotationAxis);

        this.deadband = deadband;

        goalRotation = goal;
        
        pidController = new PIDController(0.0056, 0.00, 0);
        pidController.enableContinuousInput(-180, 180);
        rightJoy = Controls.rightJoy::getAsBoolean;

        addRequirements(Swerve.get());
    }
    public TeleopSwerve(Supplier<Rotation2d> rotation){
        TeleopSwerve(()->0, ()->0, ()->0, ()->true, rotation, 0, true, false);
    }

    public TeleopSwerve(GenericHID controller, int translationAxis, int strafeAxis, int rotationAxis, double deadband,boolean fieldRelative, boolean openLoop) {
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        translation = ()->-controller.getRawAxis(translationAxis);
        strafe = ()->controller.getRawAxis(strafeAxis);
        rotation = ()->-controller.getRawAxis(rotationAxis);

        this.deadband = deadband;

        goalRotation = ()->Swerve.get().AngleForSpeaker().getDegrees();
        
        pidController = new PIDController(0.0056, 0.00, 0);
        pidController.enableContinuousInput(-180, 180);
        rightJoy = Controls.rightJoy::getAsBoolean;

        addRequirements(Swerve.get());
    }

    public TeleopSwerve(DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, BooleanSupplier rightJoy, DoubleSupplier goal, double deadband ,boolean fieldRelative, boolean openLoop) {
        translation = x;
        strafe = y;
        rotation = r;

        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;

        this.deadband = deadband; 

        this.rightJoy = rightJoy;

        this.goalRotation = goal;

        pidController = new PIDController(0.0056, 0.00, 0);
        pidController.enableContinuousInput(-180, 180);

        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double yAxis = translation.getAsDouble();
        double xAxis = strafe.getAsDouble();
        double rAxis = rotation.getAsDouble();

        Rotation2d theta = new Rotation2d(yAxis, xAxis);
        double magnitude = Math.min(Math.sqrt((xAxis * xAxis) + (yAxis * yAxis)),1);
        if (magnitude < deadband) magnitude = 0;

        if (rightJoy != null && rightJoy.getAsBoolean()){
            double pid = pidController.calculate(Swerve.get().getPose().getRotation().getDegrees(), goalRotation.getAsDouble());
            if (Math.abs(pidController.getPositionError()) > 1)
                rAxis = Math.abs(Math.pow(pid, 2)) * 0.7 * Math.signum(pid) + pid * 2.2;
            else    
                rAxis = 0;
            //magnitude *= 0.75;
        } else {
            if (Math.abs(rAxis) > deadband) {
                if (rAxis > 0)
                    rAxis = Util.map(rAxis, deadband, 1, 0, 1);
                else 
                    rAxis = Util.map(rAxis, -deadband, -1, 0, -1);
            } else {
                rAxis = 0;
            }
        }

        double rotation = rAxis * Constants.swerve.maxAngularVelocity;
        Translation2d translation = new Translation2d(Math.pow(magnitude, 2), theta).times(Constants.swerve.maxSpeed);

        Swerve.get().Drive(translation, rotation, fieldRelative, openLoop);
    }
}
