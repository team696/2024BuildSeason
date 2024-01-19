// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.Log;

public class Shooter extends SubsystemBase {
  private static Shooter m_Shooter;

  private AprilTagFieldLayout m_atLayout; 

  private TalonFX m_Master;
  private TalonFX m_Follower;

  private BangBangController m_shooterController = new BangBangController();

  private DutyCycleOut m_Request = new DutyCycleOut(0);

  public static synchronized Shooter get() {
    if (m_Shooter == null) {
      m_Shooter = new Shooter();
    }
    return m_Shooter;
  }
  
  public Shooter() {
    try {
      m_atLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      Log.fatalException("Shooter", "Failed to load april tag layout.", e);
    }

    m_Master = new TalonFX(10);
    m_Follower = new TalonFX(11);

    m_Master.getConfigurator().apply(Constants.CONFIGS.shooterMasterFXConfig);
    m_Follower.getConfigurator().apply(Constants.CONFIGS.shooterFollowerFXConfig);

    m_Follower.setControl(new Follower(m_Master.getDeviceID(), false));
  }

  /** RPM */
  public void setSpeed(double speed) {
    m_Master.setControl(m_Request.withOutput(m_shooterController.calculate(m_Master.getVelocity().getValueAsDouble() * 60.0, speed)));
  }

  public void stop() {
    m_Master.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
