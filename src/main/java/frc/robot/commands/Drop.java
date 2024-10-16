// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class Drop extends Command {
    double start = 0;
    public Drop() {
        addRequirements(Shooter.get());
    }

    public Drop(boolean finish) {
        addRequirements(Shooter.get());
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mult = (Timer.getFPGATimestamp() - start) / 1;
    Shooter.get().setAngle(30);
    Shooter.get().setSerializerSpeedPercent(.65 * mult + 0.5); 
    Shooter.get().setShooterSpeedPercent(0.25 * mult + 0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stopAngle();
    Shooter.get().stopSerializer();
    Shooter.get().stopShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
