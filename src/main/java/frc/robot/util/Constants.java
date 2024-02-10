package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class Constants {
  public static final boolean DEBUG = true;

  public static final Configs CONFIGS = new Configs();

  public static final String canivoreName = "vore";

  public static final double deadBand = 0.03;

  public static final Field2d field = new Field2d();
  public static class Motors {
    public static final class Falcon {
      public static final double stallCurA = 257;
      public static final double stallTorqueNm = 4.69;
      public static final double freeSpinRPM = 6380;
      public static final double freeSpinA = 1.5;
    }
    public static final class Kraken {
      public static final double stallCurA = 366;
      public static final double stallTorqueNm = 7.09;
      public static final double freeSpinRPM = 6000;
      public static final double freeSpinA = 1.5;
    }
  }
  public static class Cameras {
    public static final String name = "C";
    public static final Transform3d position =  new Transform3d(new Translation3d(Units.inchesToMeters(-11.441), Units.inchesToMeters(-7.051), Units.inchesToMeters(8.5)), new Rotation3d(0, Units.degreesToRadians(27.8), Units.degreesToRadians(180)));
  }
  public static class Shooter {
    public static final double AngleOffset = -67;
  }
  public static class Swerve {
    public static final double drivekS = (0.667 / 12); 
    public static final double drivekV = (2.44 / 12);
    public static final double drivekA = (0.27 / 12);

    public static final double driveGearRatio = (6.12 / 1.0); // L3
    public static final double angleGearRatio = (150.0/7.0 / 1.0); 

	public static final double massKgs = Units.lbsToKilograms(115);

    public static final double wheelX = Units.inchesToMeters(21.75);
    public static final double wheelY = Units.inchesToMeters(21.75);
    public static final double wheelDiameter = Units.inchesToMeters(3.94);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

	public static final double theoreticalMaxSpeed = Motors.Kraken.freeSpinRPM / 60 / driveGearRatio * wheelCircumference; // 5.13 mps way more resonable
	public static final double theoreticalMaxAcceleration = (4 * Motors.Kraken.stallTorqueNm * driveGearRatio) / (massKgs * wheelDiameter / 2);  // 66 mps^2 wtf
    public static final double maxSpeed = 5.16; //MPS
    public static final double maxAngularVelocity = 8; //MPS^2

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
  }
  public static class Robot {
  
      public enum Robots {
        SIM,
        UNKNOWN,
        COMP,
        BETA
      }

      public static Robots detected = Robots.UNKNOWN;

      public static final byte[] COMP_MAC = new byte[]{ (byte) 0x00, (byte) 0x80, (byte) 0x2F, (byte) 0x38, (byte) 0x5F, (byte) 0x75 };
      public static final byte[] BETA_MAC = new byte[]{ (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x35, (byte) 0xb8, (byte) 0xca };
  }

}
