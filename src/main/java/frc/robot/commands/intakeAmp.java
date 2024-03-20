package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.Position;

public class intakeAmp extends Command {
    boolean spitBack = false;

    double on = Double.MAX_VALUE;
    double off = Double.MAX_VALUE;

    boolean finish = false;

    public intakeAmp() {
        finish = false;
        addRequirements(Intake.get(), Shooter.get());
    }

    public intakeAmp(boolean finish) {
        this.finish = finish;
        addRequirements(Intake.get(), Shooter.get());
    }

    @Override 
    public void initialize() {
        LED.get().override = true;
        spitBack = !Shooter.get().getBeamBreak();
        on = Double.MAX_VALUE;
        off = Double.MAX_VALUE;
    }

    @Override
    public void execute() {
        if (!spitBack) {
            Shooter.get().stopSerializer();
            Shooter.get().setAngle(30);
            if (Intake.get().getBeamBreak()) {
                if (Intake.get().mainAngle() > 15) {
                    Intake.get().setRollersOutput(0.40);
                } else {
                    Intake.get().setRollersOutput(0);
                }
                Intake.get().positionAngle(Position.down);
                LED.get().setOverride(255, 0, 0);
            } else {
                Intake.get().setRollersOutput(0);
                Intake.get().positionAngle(Position.stowed);
                LED.get().setOverride(0, 255, 0);
            }
        } else {
            LED.get().setOverride(255, 0, 0);
            Shooter.get().setAngle(32);
            if (off == Double.MAX_VALUE && !Intake.get().getBeamBreak()) {
                off = Timer.getFPGATimestamp();
            }
            if (off != Double.MAX_VALUE && Intake.get().getBeamBreak()) {
                on = Timer.getFPGATimestamp();
            }

            if (on != Double.MAX_VALUE){
                Intake.get().setRollersOutput(0.40);
                spitBack = false;
            } else {
                Intake.get().positionAngle(Position.passback);

                if (Intake.get().mainAngle() > Position.passback.pos - 1.5) {
                    Intake.get().setRollersOutput(-0.5);
                    Intake.get().setSerializerSpeedPercent(-0.55);
                    Shooter.get().setSerializerSpeedPercent(-0.4);
                } else {
                    Intake.get().stopRollers();
                    Intake.get().stopSerializer();
                    Shooter.get().stopSerializer();
                }
            }
        }
    }

    @Override 
    public void end(boolean interrupted) {
        Intake.get().stopRollers();
        Intake.get().stopSerializer();
        Shooter.get().stopAngle();
        Shooter.get().stopSerializer();
        Shooter.get().stopShooter();
        LED.get().override = false;
    }

    @Override
    public boolean isFinished() {
        if (finish && !Intake.get().getBeamBreak())
            return true;
        return false;
    }
}
