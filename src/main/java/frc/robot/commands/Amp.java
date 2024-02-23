// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Amp extends Command {
  /** Creates a new Amp. */
  public Amp() {
    addRequirements(Shooter.get());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setAngle(65);//59
    //Shooter.get().setSpeed(650,650);
    Shooter.get().setShooterSpeedPercent(0.35);//
    if(Shooter.get().upToSpeed(650,650, 100) && !Shooter.get().getBeamBreak() && Shooter.get().atAngle(65, 2)) {
      Shooter.get().setSerializerSpeedPercent(1);
    } else {
      Shooter.get().stopSerializer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stopShooter();
    Shooter.get().stopSerializer();
    Shooter.get().stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
