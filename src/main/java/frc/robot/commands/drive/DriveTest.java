package frc.robot.commands.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveTest extends Command {
    private DriveTrain driveTrain;
    private double length;

    public DriveTest(DriveTrain driveTrain, double length) {
        this.length = length;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
      driveTrain.resetEncoders();
      
    }
    @Override
    public void execute() {
      driveTrain.diffDrive(0.4, 0.4);
      SmartDashboard.putNumber("Left Encoder Pos", driveTrain.getLeftEncoderPos());
    }

    @Override
    public boolean isFinished() {
      if (driveTrain.getLeftEncoderPos() >= length) {
        return true;
      }
      else {
        return false;
      }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
