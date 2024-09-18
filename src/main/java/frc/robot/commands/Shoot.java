// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Shoot extends Command {
    Supplier<Double> distSupplier;
    boolean finish = false;
    Supplier<Boolean> button;
    double start = 0;
    double unbroken = 0;
    double broken = 0;
    boolean feed = false;

    static DoubleSubscriber dblTopic; 

    public Shoot(Supplier<Double> distSupplier, Supplier<Boolean> button) {
        this.distSupplier = distSupplier;
        this.finish = false;
        this.button = button;
        addRequirements(Shooter.get());
    }

    public Shoot(Supplier<Double> distSupplier, boolean finish) {
        this.distSupplier = distSupplier;
        this.finish = finish;
        this.button = ()->true;
        addRequirements(Shooter.get());
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    unbroken = Double.MAX_VALUE;
    broken = Double.MAX_VALUE;
    start = Double.MAX_VALUE;//Timer.getFPGATimestamp();
    feed = false;
    dblTopic = NetworkTableInstance.getDefault().getDoubleTopic("Shooter Offset").subscribe(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   
    if (broken == Double.MAX_VALUE && !Shooter.get().getBeamBreak()) {
        broken = Timer.getFPGATimestamp();
    }
    if (feed && unbroken == Double.MAX_VALUE && Shooter.get().getBeamBreak()) {
        unbroken = Timer.getFPGATimestamp();
    }

    double actualDist = distSupplier.get();
    double ChassisX = Swerve.get().getRobotRelativeSpeeds().vxMetersPerSecond;
    Shooter.State desiredStateStationary = Shooter.get().getStateFromDist(actualDist);
    Shooter.State desiredState = desiredStateStationary;
    desiredState.angle += (actualDist * ChassisX * -.525);
    desiredState.angle += dblTopic.get();
    desiredState.angle += 2;

    Shooter.get().setAngle(desiredState.angle);
    Shooter.get().setSpeed(desiredState.topSpeed, desiredState.bottomSpeed);
    boolean aimed = Math.abs(Swerve.get().getPose().getRotation().getDegrees() - Swerve.get().AngleForSpeaker().getDegrees()) < 4;
    if(/*aimed &&*/ button.get() && Shooter.get().upToSpeed(desiredState.topSpeed, desiredState.bottomSpeed, 50) && Shooter.get().atAngle(desiredState.angle, 1)) {
      feed = true;
    } 
    feed=button.get();
    if (feed){
        if (start == Double.MAX_VALUE)
            start = Timer.getFPGATimestamp();
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
    dblTopic.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finish && Timer.getFPGATimestamp() - start > 0.6f)
        return true;
    //if (finish && Timer.getFPGATimestamp() - unbroken > 0.03) 
    //    return true;
    return false;
  }
}
