// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Log;

public class Shooter extends SubsystemBase {
  private static Shooter m_Shooter;

  private AprilTagFieldLayout m_atLayout; 

  public static synchronized Shooter get() {
    if (m_Shooter == null) {
      m_Shooter = new Shooter();
    }
    return m_Shooter;
  }
  
  public Shooter() {
    try{
      m_atLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (Exception e) {
      Log.fatalException("Shooter", "Failed to load april tag layout.", e);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
