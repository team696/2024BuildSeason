// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Amp extends Command {
  Shooter.State desiredState = new Shooter.State(60.5, 850 , 850);
    Supplier<Boolean> button;
  public Amp(Supplier<Boolean> b) {
    button = b;
    addRequirements(Shooter.get());
  }    // Use addRequirements() here to declare subsystem dependencies.


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setAngle(desiredState.angle);
    Shooter.get().setSpeed(desiredState.topSpeed,desiredState.bottomSpeed);
    if(button.get() && Shooter.get().upToSpeed(desiredState.topSpeed,desiredState.bottomSpeed, 100) && !Shooter.get().getBeamBreak() && Shooter.get().atAngle(desiredState.angle, 2)) {
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
