// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class ShooterIntake extends Command {
  /** Creates a new ShooterIntake. */
  public ShooterIntake() {
    addRequirements(Shooter.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LED.get().override = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setAngle(63);
    if (Shooter.get().getBeamBreak()) {
      Shooter.get().setShooterSpeedPercent(-0.17);
      Shooter.get().setSerializerSpeedPercent(-0.07);
      LED.get().setOverride(255, 0, 0);
    } else {
      Shooter.get().setSerializerSpeedPercent(0.05);
      Shooter.get().setShooterSpeedPercent(-0.17);
      LED.get().setOverride(0,255,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stopAngle();
    Shooter.get().stopShooter();
    Shooter.get().stopSerializer();
    LED.get().override = false;
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
