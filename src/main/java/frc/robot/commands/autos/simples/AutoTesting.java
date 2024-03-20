package frc.robot.commands.autos.simples;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.ShooterStop;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Shooter;

public class AutoTesting {
      DriveTrain driveTrain = new DriveTrain();
    Shooter Shooter = new Shooter();
    Intake Intake = new Intake();
    Lift Lift = new Lift();
     public Command getAutonomousCommand() {
        /*return new SequentialCommandGroup(
            
        new DriveEncoders(driveTrain, 0.3, 2)
        );*/
        return new SequentialCommandGroup(
            /*shoots */
            
            );
}
}
