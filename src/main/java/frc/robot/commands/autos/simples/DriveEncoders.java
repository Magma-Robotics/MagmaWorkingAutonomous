package frc.robot.commands.autos.simples;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveEncoders extends Command {
    private DriveTrain driveTrain;
    private double driveDistance;
    private double botSpeed;
    private double endDistance;
    private boolean reverse;

    public DriveEncoders(DriveTrain driveTrain, double speed, double userFeet, boolean reverse) {
        driveDistance = userFeet;
        botSpeed = speed;
        this.reverse = reverse;
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
      endDistance = 0;
      driveTrain.resetEncoders();
      //endDistance = driveTrain.getLeftEncoderPos() + driveDistance;
      if (reverse != true) {
        endDistance = 6.5;
      }
      else {
        endDistance = -6.5;
      }
    }
    @Override
    public void execute() {
      driveTrain.diffDrive(botSpeed, botSpeed);
      SmartDashboard.putNumber("Left Encoder Pos", driveTrain.getLeftEncoderPos());
    }

    @Override
    public boolean isFinished() {
      if (reverse != true) {
        return (driveTrain.getLeftEncoderPos() >= endDistance);
      }
      else {
        return (driveTrain.getLeftEncoderPos() <= endDistance);
      }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }
}
