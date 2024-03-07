// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.Position;

public class ShootIntakeAmp extends Command {
    BooleanSupplier button;
  /** Creates a new ShootIntakeAmp. */
  public ShootIntakeAmp(BooleanSupplier button) {
    this.button = button;
        addRequirements(Intake.get(), Shooter.get());
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setAngle(63);
    Intake.get().positionAngle(Position.amp);
    if (button.getAsBoolean() && Intake.get().mainAngle() > Position.amp.pos - 0.5) {
        Intake.get().setRollersOutput(-0.46);
    } else {
        Intake.get().stopRollers();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.get().stopRollers();
    Shooter.get().stopAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
