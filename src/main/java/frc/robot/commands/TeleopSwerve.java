package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.GenericHID;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TeleopSwerve extends Command {

    private boolean fieldRelative;
    private boolean openLoop;
    private DoubleSupplier translation;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private double deadband;

    ProfiledPIDController pidController;

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

        
        pidController = new ProfiledPIDController(0.03  , 0.00, 0, new TrapezoidProfile.Constraints(0.1,0.1));
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

        pidController = new ProfiledPIDController(0.018, 0.00, 0, new TrapezoidProfile.Constraints(12.,40.));
        pidController.enableContinuousInput(-180, 180);

        addRequirements(Swerve.get());
    }

    @Override
    public void initialize() {
        pidController.reset(Swerve.get().getPose().getRotation().getDegrees());
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
            rAxis = pidController.calculate(Swerve.get().getPose().getRotation().getDegrees(), Swerve.get().AngleForSpeaker().getDegrees());
            magnitude *= 0.5;
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
