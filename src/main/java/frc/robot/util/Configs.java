package frc.robot.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public SwerveModuleConstants Mod0 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod1 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod2 = new SwerveModuleConstants();
    public SwerveModuleConstants Mod3 = new SwerveModuleConstants();

    public HolonomicPathFollowerConfig FollowConfig;

    public Configs() {
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        swerveCANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        /** Swerve Angle Motor Configuration */
        swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = 40;
        swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.anglekP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.anglekI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.anglekD;

        /** Swerve Drive Motor Configuration */
        swerveDriveFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.drivekP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.drivekI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.drivekD;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        /** Individual Swerve Module Configurations */

        Mod0.CANcoderId = 5; //TODO: UPDATE ME TOO!!! :3
        Mod0.DriveMotorId = 6;
        Mod0.SteerMotorId = 7;
        Mod0.CANcoderOffset = 8;

        Mod1.CANcoderId = 9;
        Mod1.DriveMotorId = 10;
        Mod1.SteerMotorId = 11;
        Mod1.CANcoderOffset = 12;

        Mod2.CANcoderId = 13;
        Mod2.DriveMotorId = 14;
        Mod2.SteerMotorId = 15;
        Mod2.CANcoderOffset = 16;

        Mod3.CANcoderId = 17;
        Mod3.DriveMotorId = 18;
        Mod3.SteerMotorId = 19;
        Mod3.CANcoderOffset = 20;

        /** Auto Drive Config */
        FollowConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        Constants.Swerve.maxSpeed, // Max module speed, in m/s
                        Math.sqrt(Math.pow(Constants.Swerve.wheelX / 2, 2) + Math.pow(Constants.Swerve.wheelY / 2, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                );
    }
}