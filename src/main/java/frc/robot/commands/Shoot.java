// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;

public class Shoot extends Command {
    Supplier<Double> distSupplier;
    public Shoot(Supplier<Double> distSupplier) {
        this.distSupplier = distSupplier;
        addRequirements(Shooter.get());
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     *  TODO: MOVING + SHOOTING
     * 
     *  Use Distance as ratio for:
     *
     *  Swerve.get().getRobotRelativeSpeeds().vyMetersPerSecond -> Sideways speed added to angle to account for moving
     *  Swerve.get().getRobotRelativeSpeeds().vxMetersPerSecond -> Forward speed removed from speed?
     *  What To Do With Rotation Speed?
     */
    double actualDist = distSupplier.get();
    double ChassisX = Swerve.get().getRobotRelativeSpeeds().vxMetersPerSecond;
    Constants.Shooter.State desiredStateStationary = Shooter.get().getStateFromDist(actualDist);
    Constants.Shooter.State desiredState = desiredStateStationary;//Shooter.get().getStateFromDist(actualDist + (actualDist * ChassisX)/(20*Math.cos(Units.degreesToRadians(desiredStateStationary.angle)) + ChassisX)); 
    desiredState.angle = desiredState.angle + actualDist * ChassisX * -0.65;
    Shooter.get().setAngle(desiredState.angle);
    Shooter.get().setSpeed(desiredState.topSpeed, desiredState.bottomSpeed);
    if(Shooter.get().upToSpeed(desiredState.topSpeed, desiredState.bottomSpeed, 100) && !Shooter.get().getBeamBreak() && Shooter.get().atAngle(desiredState.angle, 2)) {
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
