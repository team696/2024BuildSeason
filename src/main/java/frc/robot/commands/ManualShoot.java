// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ManualShoot extends Command {
    boolean feed = false;
    public ManualShoot() {
        addRequirements(Shooter.get());
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    Shooter.State desiredState = Shooter.get().getStateFromDist(2);

    Shooter.get().setAngle(desiredState.angle);
    Shooter.get().setSpeed(desiredState.topSpeed, desiredState.bottomSpeed);
    if(Shooter.get().upToSpeed(desiredState.topSpeed, desiredState.bottomSpeed, 50) && Shooter.get().atAngle(desiredState.angle, 1)) {
      feed = true;
    } 
    if (feed){
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
