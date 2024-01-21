package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Camera;
import frc.robot.util.Constants;
import frc.robot.util.SwerveModule;

public class Swerve extends SubsystemBase {
  
  private static Swerve m_Swerve;
  private AHRS m_Gyro;
  //private Pigeon2 m_Pigeon;

  private SwerveModulePosition[] m_swervePositions = new SwerveModulePosition[4];
  private SwerveDrivePoseEstimator m_poseEstimator;

  public static synchronized Swerve get() {
    if (m_Swerve == null) {
      m_Swerve = new Swerve();
    }
    return m_Swerve;
  }
  
  private Swerve() {
    m_Gyro = new AHRS(SPI.Port.kMXP);
    zeroYaw();

    //m_Pigeon = new Pigeon2(0);
    //m_Pigeon.getConfigurator().apply(Constants.CONFIGS.swervePigeon2Configuration);

   for (int i = 0; i < 4; ++i) {
      m_swervePositions[i] = Constants.Swerve.swerveMods[i].getPosition();
    }

    m_poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), m_swervePositions, new Pose2d(0,0,new Rotation2d(0)));
  }

  /**  -180 , 180 */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-1 * m_Gyro.getYaw()); 
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void zeroYaw() {
    m_Gyro.zeroYaw();
  }

  public void resetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getYaw(), m_swervePositions, pose);
  }

  public void resetPose(){
    resetPose(new Pose2d());
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }
  
  private SwerveModuleState[] getStates() { 
    SwerveModuleState[] states = new SwerveModuleState[4]; 
    for(SwerveModule mod : Constants.Swerve.swerveMods) { 
      states[mod.moduleNumber] = mod.getState(); 
    } 
    return states; 
  }

  public void Drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            ) : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for(SwerveModule mod : Constants.Swerve.swerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }  

  public void Drive(ChassisSpeeds c) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(c);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for(SwerveModule mod : Constants.Swerve.swerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
    }
  } 

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    
    for(SwerveModule mod : Constants.Swerve.swerveMods){
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  } 

  public Rotation2d getAngleForSpeaker(Pose2d  pose) {
    return Rotation2d.fromRadians(Math.PI + (Math.atan2(pose.getY() - 5.6, pose.getX() - 1.1)));
  }

  @Override
  public void periodic() {
    for (int i = 0; i < 4; ++i) {
      m_swervePositions[i] = Constants.Swerve.swerveMods[i].getPosition();
    }

    m_poseEstimator.update(getYaw(), m_swervePositions);

    Camera.get().updatePose(m_poseEstimator);

    Constants.field.setRobotPose(getPose());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    for(SwerveModule mod : Constants.Swerve.swerveMods){
      builder.addDoubleProperty("Mod " + mod.moduleNumber + " Cancoder", ()->mod.getCANcoder().getRotations(), null);
      builder.addDoubleProperty("Mod " + mod.moduleNumber + " Internal Encoder", ()->mod.getState().angle.getRotations(),null);
    }
    builder.addDoubleProperty("Gyro", ()->getYaw().getDegrees(), null);
    
    //builder.addDoubleProperty("Pigeon", ()->m_Pigeon.getYaw().getValueAsDouble(), null);
    SmartDashboard.putData("Field", Constants.field);
  }
}
