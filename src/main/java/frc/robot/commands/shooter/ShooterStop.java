package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShooterStop extends Command {


    /**
     * an instance of {@link frc.robot.subsystems.ShooterMotor2}
     */
    private final Shooter shooter;

    /**
     * @param Shooter an instance of {@link frc.robot.subsystems.ShooterMotor2}
     */
    public ShooterStop(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }


    @Override
    public void initialize() {
    }


    /**
     * method that's being executed
     */
    @Override
    public void execute() {
        shooter.stopShooter();
    }


    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
        return false;
    }


}