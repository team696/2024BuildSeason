// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class intake extends Command {
    boolean finish = false;
    public intake() {
        this.finish = false;
        addRequirements(Shooter.get(), Intake.get());
    }

    public intake(boolean finish) {
        this.finish = finish;
        addRequirements(Shooter.get(), Intake.get());
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setAngle(50);
    if (Shooter.get().getBeamBreak()) {
        Shooter.get().setSerializerSpeedPercent(0.25); 
        Intake.get().runRollers(0.6);
    } else {
        Shooter.get().setSerializerSpeedPercent(0); 
        Intake.get().runRollers(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stopAngle();
    Shooter.get().stopSerializer();
    Intake.get().stopRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finish && !Shooter.get().getBeamBreak())
        return true;
    return false;
  }
}
