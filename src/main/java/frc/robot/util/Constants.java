// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class Constants {
  public static final Configs CONFIGS = new Configs();

  public static final double deadBand = 0.03;

  public static class Swerve {
    public static final double drivekS = (0.667 / 12); 
    public static final double drivekV = (2.44 / 12);
    public static final double drivekA = (0.27 / 12);

    public static final double drivekP = 0.1; 
    public static final double drivekI = 0.;
    public static final double drivekD = 0.;
    public static final double anglekP = 0.6; 
    public static final double anglekI = 0.;
    public static final double anglekD = 12.;

    public static final double driveGearRatio = (6.12 / 1.0); // L3
    public static final double angleGearRatio = (12.8 / 1.0); 

    public static final double maxSpeed = 5.16; //MPS
    public static final double maxAngularVelocity = 8; //MPS^2

    public static final double wheelX = Units.inchesToMeters(24); //TODO: UPDATE ME
    public static final double wheelY = Units.inchesToMeters(24);
    public static final double wheelDiameter = Units.inchesToMeters(3.94);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    private static final SwerveModule frontLeft = new SwerveModule(0, Constants.CONFIGS.Mod0);
    private static final SwerveModule frontRight = new SwerveModule(1, Constants.CONFIGS.Mod1);
    private static final SwerveModule backLeft = new SwerveModule(2, Constants.CONFIGS.Mod2);
    private static final SwerveModule backRight = new SwerveModule(3, Constants.CONFIGS.Mod3);
    public static final SwerveModule[] swerveMods = { frontLeft, frontRight, backLeft, backRight };

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelX / 2.0, wheelY / 2.0), // FL
        new Translation2d(wheelX / 2.0, -wheelY / 2.0), // FR
        new Translation2d(-wheelX / 2.0, wheelY / 2.0), // BL
        new Translation2d(-wheelX / 2.0, -wheelY / 2.0)); // BR
    
    public static final Field2d field = new Field2d();
  }
}
