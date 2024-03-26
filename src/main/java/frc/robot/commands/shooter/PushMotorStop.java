package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PushMotorStop extends Command {
    public Shooter shooter;

    public PushMotorStop(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.stopPushMotor();
    }


    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
