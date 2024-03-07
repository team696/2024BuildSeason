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
    double goalPos = 0;
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
    goalPos = Shooter.get().getSerializerPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (finish)
        Shooter.get().setShooterSpeedPercent(0.45);
    else 
        Shooter.get().setShooterSpeedPercent(-0.03);
    Shooter.get().setAngle(32);
    if (Shooter.get().getBeamBreak() ) {
        Shooter.get().setSerializerSpeedPercent(0.06);
        if (Intake.get().mainAngle() > 15) {
            Intake.get().setSerializerSpeedPercent(0.5); 
            Intake.get().setRollersOutput(0.8);
        }
        Intake.get().positionAngle(Position.down);
        LED.get().setOverride(255,0,0);
        goalPos = Shooter.get().getSerializerPosition() + 0.25;

    } else {
        
        Shooter.get().serializerPosition(goalPos);
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
    LED.get().override = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finish && broken != 0 && Timer.getFPGATimestamp() - broken > 0.03)
        return true;
    return false;
  }
}
