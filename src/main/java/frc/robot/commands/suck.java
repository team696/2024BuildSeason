// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.Position;

public class suck extends Command {
  /** Creates a new suck. */
  public suck() {
    addRequirements(Intake.get());
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if (Intake.get().mainAngle() > 15) {
            Intake.get().setSerializerSpeedPercent(0.25); 
            Intake.get().setRollersOutput(0.8);
        }
        Intake.get().positionAngle(Position.down);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.get().stopRollers();
    Intake.get().stopSerializer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
