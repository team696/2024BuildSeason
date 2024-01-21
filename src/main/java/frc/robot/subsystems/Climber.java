// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static Climber m_Climber;

  public static synchronized Climber get() {
    if (m_Climber == null) {
      m_Climber = new Climber();
    }
    return m_Climber;
  }

  /** Creates a new Climber. */
  private Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  @Override
  public void initSendable(SendableBuilder builder) {

  }
}
