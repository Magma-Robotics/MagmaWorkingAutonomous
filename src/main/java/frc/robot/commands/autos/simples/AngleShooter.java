package frc.robot.commands.autos.simples;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;


public class AngleShooter extends Command {
    private Pivot pivot;
    private double botSpeed;
    private double endDegrees;


    public AngleShooter(Pivot pivot, double botSpeed, double endDegrees) {
        this.endDegrees = endDegrees;
        this.botSpeed = botSpeed;
        this.pivot = pivot;
        addRequirements(pivot);
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
      SmartDashboard.putNumber("LeftPivot Encoder Pos", pivot.getLeftPivotEncoderPos());
    }


    @Override
    public boolean isFinished() {
      if (botSpeed > 0) {
        return pivot.getLeftPivotEncoderPos() >= endDegrees;
      }
      else {
        return (pivot.getLeftPivotEncoderPos() <= endDegrees);
      }
    }


    @Override
    public void end(boolean interrupted) {
        pivot.PivotStop();
    }
}

