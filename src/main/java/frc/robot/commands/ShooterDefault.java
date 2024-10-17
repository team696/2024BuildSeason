// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterDefault extends Command {
    double goal = 0;
  /** Creates a new ShooterDefault. */
    public ShooterDefault() {
        addRequirements(Shooter.get());
    }

    @Override
    public void initialize() {
        goal = Shooter.get().getSerializerPosition();
    }

    @Override
    public void execute() {
        Shooter.get().setAngle(30);
        Shooter.get().serializerPosition(goal);
        if (DriverStation.isAutonomousEnabled()) {
            Shooter.get().setShooterSpeedPercent(0);
        } else {
            Shooter.get().setShooterSpeedPercent(0.05);
        }
    }

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
