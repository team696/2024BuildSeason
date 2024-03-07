package frc.robot.util;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
    public TalonFXConfiguration intake_Serializer = new TalonFXConfiguration();
    public TalonFXConfiguration intake_Rollers = new TalonFXConfiguration();

    public TalonFXConfiguration climber_Master = new TalonFXConfiguration();
    public TalonFXConfiguration climber_Follower = new TalonFXConfiguration();

    public CANdleConfiguration candle = new CANdleConfiguration();

    public Configs() {
        /** Swerve CANCoder Configuration */
        swerve_CANCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        swerve_CANCoder.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        /** Swerve Angle Motor Configuration */
        swerve_Angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerve_Angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        swerve_Angle.Feedback.SensorToMechanismRatio = Constants.swerve.angleGearRatio;
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
        swerve_Drive.Feedback.SensorToMechanismRatio = Constants.swerve.driveGearRatio;
        swerve_Drive.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerve_Drive.CurrentLimits.SupplyCurrentLimit = 35;
        swerve_Drive.CurrentLimits.SupplyCurrentThreshold = 60;
        swerve_Drive.CurrentLimits.SupplyTimeThreshold = 0.1;
        swerve_Drive.Slot0.kP = 2.;
        swerve_Drive.Slot0.kI = 0.0;
        swerve_Drive.Slot0.kD = 0.0;
        swerve_Drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2;
        swerve_Drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2;
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

        /** Shooter Motor Configuration */
        shooter_Top.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_Top.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooter_Top.Slot0.kP = 1;
        shooter_Top.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooter_Top.CurrentLimits.SupplyCurrentLimit = 35;
        shooter_Top.CurrentLimits.SupplyCurrentThreshold = 50;
        shooter_Top.CurrentLimits.SupplyTimeThreshold = 0.1;
        
    
        shooter_Bottom.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_Bottom.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooter_Bottom.Slot0.kP = 1;
        shooter_Bottom.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooter_Bottom.CurrentLimits.SupplyCurrentLimit = 35;
        shooter_Bottom.CurrentLimits.SupplyCurrentThreshold = 50;
        shooter_Bottom.CurrentLimits.SupplyTimeThreshold = 0.1;
        
        shooter_Serializer.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooter_Serializer.Slot0.kP = 2.5;

        shooter_Angle.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shooter_Angle.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooter_Angle.Feedback.SensorToMechanismRatio = 94.5;

        shooter_Angle.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooter_Angle.CurrentLimits.SupplyCurrentLimit = 20;
        shooter_Angle.CurrentLimits.SupplyCurrentThreshold = 30;
        shooter_Angle.CurrentLimits.SupplyTimeThreshold = 0.1;

        /** Pigeon Configuration */ 
        swerve_Pigeon.MountPose.MountPoseYaw = 0;

        /** Climber Motor Configuration */
        climber_Master.Slot0.kP = 1;
        climber_Master.Feedback.SensorToMechanismRatio = 27;
        climber_Master.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        climber_Follower.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /** Intake Motor Configuaration */
        intake_Angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intake_Angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intake_Angle.Slot0.kP = 1/11.0;
        intake_Angle.MotionMagic.MotionMagicAcceleration = 256;
        intake_Angle.MotionMagic.MotionMagicCruiseVelocity = 128;

        intake_Angle.CurrentLimits.SupplyCurrentLimitEnable = true;
        intake_Angle.CurrentLimits.SupplyCurrentLimit = 20;
        intake_Angle.CurrentLimits.SupplyCurrentThreshold = 30;
        intake_Angle.CurrentLimits.SupplyTimeThreshold = 0.;

        //intake_Angle.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;

        intake_Serializer.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        /** Candle Configuration */
        candle.statusLedOffWhenActive = true;
        candle.disableWhenLOS = false;
        candle.stripType = LEDStripType.RGB;
        candle.brightnessScalar = 1;
        candle.vBatOutputMode = VBatOutputMode.On;
        candle.enableOptimizations = true;
        candle.v5Enabled = true;
    }
}