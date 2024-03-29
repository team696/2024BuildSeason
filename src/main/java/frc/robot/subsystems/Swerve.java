package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.SubsystemHandler;
import frc.lib.Log.Debug;
import frc.lib.Log.Log;
import frc.lib.Log.PLog;
import frc.robot.util.Camera;
import frc.robot.util.Constants;
import frc.robot.util.SwerveModule;

public class Swerve extends SubsystemHandler {
  
  private static Swerve m_Swerve;
  //private AHRS m_Gyro;

  private Pigeon2 m_Pigeon; 

  private SwerveModulePosition[] m_swervePositions = new SwerveModulePosition[4];
  private SwerveDrivePoseEstimator m_poseEstimator;

  private double accelerationX = 0;
  private double accelerationY = 0;

  private double oldAccelerationX = 0;
  private double oldAccelerationY = 0;

  private double lastTimestamp = 0;

  public static Swerve get() {
    if (m_Swerve == null) {
      m_Swerve = new Swerve();
    }
    return m_Swerve;
  }
  
  private Swerve() {
    for (int i = 0; i < 4; ++i) {
      m_swervePositions[i] = Constants.swerve.swerveMods[i].getPosition();
    }

    //m_Gyro = new AHRS(SPI.Port.kMXP);
    m_Pigeon = new Pigeon2(0);
    m_Pigeon.getConfigurator().apply(Constants.CONFIGS.swerve_Pigeon);

    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.swerve.swerveKinematics, getYaw(), m_swervePositions, new Pose2d(0,0,new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.03), VecBuilder.fill(0.3, 0.3, 0.6)); 
  
    zeroYaw();
    }


 /**  -180 , 180 */ @Log
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-1 * m_Pigeon.getAngle()); 
  }

  @Debug
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void zeroYaw() {
    m_Pigeon.reset();
    //m_Gyro.zeroYaw();
    resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0) : 0))));
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getYaw(), m_swervePositions, pose);
  }

  public void resetPose(){
    resetPose(new Pose2d());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  private SwerveModuleState[] getStates() { 
    SwerveModuleState[] states = new SwerveModuleState[4]; 
    for(SwerveModule mod : Constants.swerve.swerveMods) { 
      states[mod.moduleNumber] = mod.getState(); 
    } 
    return states; 
  }

  public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getPose().getRotation().rotateBy(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0) : 0)))
                            ) : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.swerve.maxSpeed);

    for(SwerveModule mod : Constants.swerve.swerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void Drive(ChassisSpeeds c) {
    SwerveModuleState[] swerveModuleStates = Constants.swerve.swerveKinematics.toSwerveModuleStates(c);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.swerve.maxSpeed);

    for(SwerveModule mod : Constants.swerve.swerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
    }
  } 

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.swerve.maxSpeed);
    
    for(SwerveModule mod : Constants.swerve.swerveMods){
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  } 

  public Rotation2d AngleForSpeaker() {
    Translation2d delta;
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        delta = getPose().getTranslation().minus(Constants.Field.RED.Speaker);
        delta = delta.minus(new Translation2d(0, (getRobotRelativeSpeeds().vyMetersPerSecond * 1/8)  * delta.getNorm()).rotateBy(getPose().getRotation().plus(Rotation2d.fromDegrees(180))));  
        return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX())).rotateBy(new Rotation2d(Math.PI));
    } else {
        delta = getPose().getTranslation().minus(Constants.Field.BLUE.Speaker);
        delta = delta.plus(new Translation2d(0, (getRobotRelativeSpeeds().vyMetersPerSecond * 1/8)  * delta.getNorm()).rotateBy(getPose().getRotation()));  
        return Rotation2d.fromRadians(Math.atan(delta.getY() / delta.getX()));
    }                                                                   
  }

  @Debug
  public double DistToSpeaker() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return getPose().getTranslation().getDistance(Constants.Field.RED.Speaker);
    } else {
        return getPose().getTranslation().getDistance(Constants.Field.BLUE.Speaker);
    }
  }

  @Override
  public void periodic() {

    //if (!m_Gyro.isConnected())
    //  PLog.unusual("Swerve", "Gyro Not Found");

    for (int i = 0; i < 4; ++i) {
      m_swervePositions[i] = Constants.swerve.swerveMods[i].getPosition();
    }

    m_poseEstimator.update(getYaw(), m_swervePositions);

    //Potentially incorporate gyro?
    accelerationX = (getRobotRelativeSpeeds().vxMetersPerSecond - oldAccelerationX) / (Timer.getFPGATimestamp() - lastTimestamp);
    accelerationY = (getRobotRelativeSpeeds().vyMetersPerSecond - oldAccelerationY) / (Timer.getFPGATimestamp() - lastTimestamp);
    oldAccelerationX = getRobotRelativeSpeeds().vxMetersPerSecond;
    oldAccelerationY = getRobotRelativeSpeeds().vyMetersPerSecond;
    lastTimestamp = Timer.getFPGATimestamp();

    if (!DriverStation.isAutonomousEnabled()) {
        Camera.get().updatePose(m_poseEstimator);
    }

    Constants.Field.sim.setRobotPose(getPose());
    Constants.Field.sim.getObject("robot").setPose(getPose());
    Constants.Field.sim.getObject("cam").setPose(getPose().transformBy(new Transform2d(Constants.Cameras.position.getTranslation().toTranslation2d(), Constants.Cameras.position.getRotation().toRotation2d())));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if ( Constants.DEBUG) {
      for(SwerveModule mod : Constants.swerve.swerveMods){
        builder.addDoubleProperty("Mod " + mod.moduleNumber + " Cancoder", ()->mod.getCANCoderAngle().getRotations(), null);
      }
      builder.addDoubleProperty("Gyro", ()->getYaw().getDegrees(), null);
    }

    builder.addDoubleProperty("Distance To Speaker", ()->DistToSpeaker(), null);
    builder.addDoubleProperty("angle for speaker", ()->AngleForSpeaker().getDegrees(), null);
    builder.addDoubleProperty("AccelerationX", ()->accelerationX, null); 
    builder.addDoubleProperty("AccelerationY", ()->accelerationY, null); 

    builder.addStringProperty("Chassis Speeds", ()->getRobotRelativeSpeeds().toString(), null);

    SmartDashboard.putData("Field", Constants.Field.sim);
  }
}
