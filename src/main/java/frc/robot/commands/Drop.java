// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Drop extends Command {
    public Drop() {
        addRequirements(Shooter.get(), Intake.get());
    }

    public Drop(boolean finish) {
        addRequirements(Shooter.get(), Intake.get());
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setAngle(30);
    Shooter.get().setSerializerSpeedPercent(.65); 
    Shooter.get().setShooterSpeedPercent(0.25);

    Intake.get().goalAngle(4);

    if (Intake.get().mainAngle() > 2) {
        Intake.get().setRollersOutput(-0.4);
        Intake.get().setSerializerSpeedPercent(-0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stopAngle();
    Shooter.get().stopSerializer();
    Shooter.get().stopShooter();

    Intake.get().stopRollers();
    Intake.get().stopAngle();
    Intake.get().stopSerializer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
