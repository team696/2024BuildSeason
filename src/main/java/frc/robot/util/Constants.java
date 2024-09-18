package frc.robot.util;

import java.util.TreeMap;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Shooter;

public final class Constants {
	public static final boolean DEBUG = true;

    public static final Configs CONFIGS = new Configs();

	public static final String canivoreName = "vore";

	public static final double deadBand = 0.03;
	public static final class Field {
		public static final Field2d sim = new Field2d();
        public static final class RED {
		    public static final Translation2d Speaker = new Translation2d(16.58, 5.55);
            public static final Pose2d Amp = new Pose2d(12, 8, new Rotation2d(Math.PI/2));
            public static final Pose2d Source = new Pose2d(1, 0.5, Rotation2d.fromDegrees(-135));
            public static final Pose2d Trap = new Pose2d(11.754, 5.12, Rotation2d.fromDegrees(60));
        }
        public static final class BLUE {
            public static final Translation2d Speaker = new Translation2d(0, 5.55);
            public static final Pose2d Amp = new Pose2d(4, 8, new Rotation2d(Math.PI/2));
            public static final Pose2d Source = new Pose2d(15.15, 1.5, Rotation2d.fromDegrees(135)); 
        }
	}
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
		public static final Transform3d position =  new Transform3d(new Translation3d(Units.inchesToMeters(-11.441), Units.inchesToMeters(-7.051), Units.inchesToMeters(8.5)), new Rotation3d(0, Units.degreesToRadians(-27.8), Units.degreesToRadians(180)));
	}
	public static class shooter {
	    public static final double AngleOffset = /*111.5*/109;
		public static final TreeMap<Double, Shooter.State> distToState = new TreeMap<Double, Shooter.State>(){{
			put(1.5 ,new Shooter.State(/*60*/58,2500,2000));
            put(2.2 ,new Shooter.State(50, 3000,2500));
            put(2.8 ,new Shooter.State(40.5, 3000,2500));
            put(3.6 ,new Shooter.State(37.5, 3000,2500));
            put(4.3 ,new Shooter.State(35, 3000,2500));
            put(4.8, new Shooter.State(33, 3200,2900));
            put(5.2, new Shooter.State(33, 3200, 2900));
            put(5.4 ,new Shooter.State(32.5, 3200,2900));
            put(5.6, new Shooter.State(32, 3200,2900));
            put(5.7, new Shooter.State(31.5, 3200, 2900));
            put(6.2, new Shooter.State(30, 3200, 2900));
			put(12., new Shooter.State(29, 3200,2900));
         /*
            put(1., new Shooter.State(60, 3000, 3000));
            put(1.5, new Shooter.State(60, 3000, 3000));
            put(2., new Shooter.State(50, 3000, 3000));
            put(2.5, new Shooter.State(45, 3000, 3000));
            put(3., new Shooter.State(42, 3000, 3000));
            put(3.5, new Shooter.State(39, 3000, 3000));
            put(4., new Shooter.State(36, 3000, 3000));
            put(4.5, new Shooter.State(3 5, 3000, 3000));
            put(5., new Shooter.State(33, 3000, 3000));
            put(5.5, new Shooter.State(33, 3000, 3000));
            put(6., new Shooter.State(31, 3000, 3000));

            put(12., new Shooter.State(32, 3000, 3000));
   */
		}};
	}
	public static class swerve {
		public static final double drivekS = (0.667 / 12); 
		public static final double drivekV = (2.44 / 12);
		public static final double drivekA = (0.27 / 12);

		public static final double driveGearRatio = (6.12 / 1.0); // L3
		public static final double angleGearRatio = (150.0/7.0 / 1.0); 

		public static final double massKgs = Units.lbsToKilograms(115);

		public static final double wheelX = Units.inchesToMeters(21.75);
		public static final double wheelY = Units.inchesToMeters(21.75);
		public static final double wheelDiameter = Units.inchesToMeters(3.89);
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double theoreticalMaxSpeed = Motors.Kraken.freeSpinRPM / 60 / driveGearRatio * wheelCircumference; // 5.13 mps way more resonable
		public static final double theoreticalMaxAcceleration = (4 * Motors.Kraken.stallTorqueNm * driveGearRatio) / (massKgs * wheelDiameter / 2);  // 66 mps^2 wtf
		public static final double maxSpeed = 5.13; //MPS
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

    public static final class Auto {
        public static final HolonomicPathFollowerConfig FollowConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            Constants.swerve.maxSpeed, // Max module speed, in m/s
            Math.sqrt(Math.pow(Constants.swerve.wheelX / 2, 2) + Math.pow(Constants.swerve.wheelY / 2, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        );
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
