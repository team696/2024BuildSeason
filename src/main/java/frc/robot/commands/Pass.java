// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants.shooter;

public class Pass extends Command {
  boolean feed;
  boolean didSeeFront=false;
  double didUnseeFront=0;

  /** Creates a new Pass. */
  public Pass() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Shooter.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feed=false;
    didSeeFront=false;
    didUnseeFront=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double dist=Swerve.get().
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stopSerializer();
    Shooter.get().stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Shooter.get().getBeamBreak();
  }
}
