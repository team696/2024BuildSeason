package frc.robot.util;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private double m_lastAngle;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.drivekS, Constants.Swerve.drivekV, Constants.Swerve.drivekA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.CANcoderOffset; 
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.CANcoderId);
        angleEncoder.getConfigurator().apply(Constants.CONFIGS.swerve_CANCoder);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.SteerMotorId);
        mAngleMotor.getConfigurator().apply(Constants.CONFIGS.swerve_Angle);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.DriveMotorId);
        mDriveMotor.getConfigurator().apply(Constants.CONFIGS.swerve_Drive);
        mDriveMotor.getConfigurator().setPosition(0.0);

        m_lastAngle = getState().angle.getRotations();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        double angle = ((Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? m_lastAngle : desiredState.angle.getRotations());
        mAngleMotor.setControl(anglePosition.withPosition(angle));
        setSpeed(desiredState, isOpenLoop);
        m_lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Util.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset;
        mAngleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Util.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Util.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
        );
    }
}