package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Configs {

    public TalonFXConfiguration swerve_Angle = new TalonFXConfiguration();
    public TalonFXConfiguration swerve_Drive = new TalonFXConfiguration();
    public CANcoderConfiguration swerve_CANCoder = new CANcoderConfiguration();
    public Pigeon2Configuration swerve_Pigeon = new Pigeon2Configuration();
    public SwerveModuleConstants Mod0 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod1 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod2 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod3 = new SwerveModuleConstants();
    
    public TalonFXConfiguration shooter_Top = new TalonFXConfiguration();
    public TalonFXConfiguration shooter_Bottom = new TalonFXConfiguration();
    public TalonFXConfiguration shooter_Angle = new TalonFXConfiguration();
    public TalonFXConfiguration shooter_Serializer = new TalonFXConfiguration();

    public TalonFXConfiguration intake_Angle = new TalonFXConfiguration();
    public TalonFXConfiguration intake_Linear = new TalonFXConfiguration();
    public TalonFXConfiguration intake_Rollers = new TalonFXConfiguration();
    public TalonFXConfiguration intake_SecondAngle = new TalonFXConfiguration();


    public TalonFXConfiguration climber_Master = new TalonFXConfiguration();
    public TalonFXConfiguration climber_Follower = new TalonFXConfiguration();

    public HolonomicPathFollowerConfig FollowConfig;

    public Configs() {
        /** Swerve CANCoder Configuration */
        swerve_CANCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        swerve_CANCoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        /** Swerve Angle Motor Configuration */
        swerve_Angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerve_Angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        swerve_Angle.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerve_Angle.ClosedLoopGeneral.ContinuousWrap = true;
        swerve_Angle.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerve_Angle.CurrentLimits.SupplyCurrentLimit = 25;
        swerve_Angle.CurrentLimits.SupplyCurrentThreshold = 40;
        swerve_Angle.CurrentLimits.SupplyTimeThreshold = 0.1;
        swerve_Angle.Slot0.kP = 128.0;
        swerve_Angle.Slot0.kI = 0.0;
        swerve_Angle.Slot0.kD = 6.0;

        /** Swerve Drive Motor Configuration */
        swerve_Drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerve_Drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swerve_Drive.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;
        swerve_Drive.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerve_Drive.CurrentLimits.SupplyCurrentLimit = 35;
        swerve_Drive.CurrentLimits.SupplyCurrentThreshold = 60;
        swerve_Drive.CurrentLimits.SupplyTimeThreshold = 0.1;
        swerve_Drive.Slot0.kP = 2.;
        swerve_Drive.Slot0.kI = 0.0;
        swerve_Drive.Slot0.kD = 0.0;
        swerve_Drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.3;
        swerve_Drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;
        swerve_Drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        swerve_Drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        /** Individual Swerve Module Configurations -> frontLeft, frontRight, backLeft, backRight */ 
        Mod0.CANcoderId = 0;  
        Mod0.DriveMotorId = 1; 
        Mod0.SteerMotorId = 2;
        Mod0.CANcoderOffset = -0.2539;
 
        Mod1.CANcoderId = 2; 
        Mod1.DriveMotorId = 3;
        Mod1.SteerMotorId = 4;
        Mod1.CANcoderOffset = -0.433;

        Mod2.CANcoderId = 1; 
        Mod2.DriveMotorId = 5;
        Mod2.SteerMotorId = 6;
        Mod2.CANcoderOffset = 0.3265;

        Mod3.CANcoderId = 3; 
        Mod3.DriveMotorId = 7;
        Mod3.SteerMotorId = 8;
        Mod3.CANcoderOffset = -0.15845;

        /** Auto Drive Config */
        FollowConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        Constants.Swerve.maxSpeed, // Max module speed, in m/s
                        Math.sqrt(Math.pow(Constants.Swerve.wheelX / 2, 2) + Math.pow(Constants.Swerve.wheelY / 2, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                );

        /** Shooter Motor Configuration */
        shooter_Top.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_Top.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        shooter_Bottom.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_Bottom.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooter_Serializer.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooter_Angle.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooter_Angle.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /** Pigeon Configuration */ 
        swerve_Pigeon.MountPose.MountPoseYaw = 0;

        /** Climber Motor Configuration */
        climber_Master.Slot0.kP = 1;
        climber_Master.Feedback.SensorToMechanismRatio = 27;

        /** Intake Motor Configuaration */
        intake_Linear.Slot0.kP = 1;
        intake_Linear.Feedback.SensorToMechanismRatio = 26;
    }
}