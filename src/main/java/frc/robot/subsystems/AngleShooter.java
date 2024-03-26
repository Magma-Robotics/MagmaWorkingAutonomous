package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;


public class AngleShooter extends Command {
    private Pivot pivot;
    private double botSpeed;
    private double endDegrees;


    public AngleShooter(Pivot Pivot, double botSpeed, double endDegrees) {
        this.endDegrees = endDegrees;
        this.botSpeed = botSpeed;
        addRequirements(Pivot);
    }

    @Override
    public void initialize() {
      pivot.resetEncoders();
      if (botSpeed > 0) {
      }
      else {
        endDegrees = -endDegrees;
      }
    }
    @Override
    public void execute() {
      SmartDashboard.putNumber("Left Encoder Pos", pivot.getPivotMotorEncoderPos());
    }


    @Override
    public boolean isFinished() {
      if (botSpeed > 0) {
        return pivot.getPivotMotorEncoderPos() >= endDegrees;
      }
      else {
        return (pivot.getPivotMotorEncoderPos() <= endDegrees);
      }
    }


    @Override
    public void end(boolean interrupted) {
        pivot.PivotStop();
    }
}

