// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.Position;

public class intake extends Command {
    boolean finish = false;
    double broken = 0;
    double start = 0;
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
  public void initialize() {
    LED.get().override = true;
    broken = 0;
    start = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Shooter.get().setShooterSpeedPercent(-0.1);
    Shooter.get().setAngle(32);
    if (Shooter.get().getBeamBreak() ) {
        Shooter.get().setSerializerSpeedPercent(0.15); 
        if (Intake.get().mainAngle() > 5) {
            Intake.get().setSerializerSpeedPercent(0.85); 
            Intake.get().setRollersOutput(0.6);
        }
        Intake.get().positionAngle(Position.down);
        LED.get().setOverride(255,0,0);
    } else {
        //if ((Timer.getFPGATimestamp() * 2) % 2 > 1) {
        //    Shooter.get().setSerializerSpeedPercent(-0.05); 
        //} else {
        //    Shooter.get().setSerializerSpeedPercent(0.05);
        //}
        Shooter.get().setSerializerSpeedPercent(0);
        Intake.get().setRollersOutput(0);
        Intake.get().setSerializerSpeedPercent(0);
        Intake.get().positionAngle(Position.stowed);

        LED.get().setOverride(0,255,0);
    }

    if (!Shooter.get().getBeamBreak()) {
        if (broken == 0) {
            broken = Timer.getFPGATimestamp();
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.get().stopAngle();
    Shooter.get().stopSerializer();
    Shooter.get().stopShooter();
    Intake.get().stopRollers();
    Intake.get().stopSerializer();
    Intake.get().stopAngle();
    LED.get().override = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finish && broken != 0 && Timer.getFPGATimestamp() - broken > 0.015)
        return true;
    return false;
  }
}
