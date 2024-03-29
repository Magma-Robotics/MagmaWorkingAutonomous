// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.Math;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Pivot extends SubsystemBase {


    /**
     * an abstract representation of a physical drive motor
     */

    private CANSparkMax leftPivotMotor, rightPivotMotor;

    private RelativeEncoder leftPivotEncoder, rightPivotEncoder;
    private static final double kDt = 0.02;
    private ProfiledPIDController pivotPIDController = 
        new ProfiledPIDController(
            Constants.Subsystems.Pivot.kP, Constants.Subsystems.Pivot.kI, Constants.Subsystems.Pivot.kD, 
            new TrapezoidProfile.Constraints(
                20, 
                20),
            kDt);
    private SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(Constants.Subsystems.Pivot.kS, Constants.Subsystems.Pivot.kV, Constants.Subsystems.Pivot.kA);
    private double appliedOutput = 0;


    public Pivot() {

        this.leftPivotMotor = new CANSparkMax(Constants.Subsystems.Pivot.kLeftPivotId, MotorType.kBrushless);
        this.rightPivotMotor = new CANSparkMax(Constants.Subsystems.Pivot.kRightPivotId, MotorType.kBrushless);

        this.leftPivotEncoder = this.leftPivotMotor.getEncoder();
        rightPivotEncoder = rightPivotMotor.getEncoder();
    
        this.leftPivotMotor.restoreFactoryDefaults();
        rightPivotMotor.restoreFactoryDefaults();

        rightPivotMotor.follow(leftPivotMotor, true);

        leftPivotMotor.setInverted(false);
        rightPivotMotor.setInverted(false);
        
        this.leftPivotMotor.burnFlash();
        rightPivotMotor.burnFlash();
    
        resetEncoders();
        SmartDashboard.putNumber("Left Pivot", this.leftPivotEncoder.getPosition());
        SmartDashboard.putData("Pivot PID", pivotPIDController);
    }
    
    public void PivotStop() {
        this.leftPivotMotor.stopMotor();
        rightPivotMotor.stopMotor();
    }

    public void resetEncoders() {
        leftPivotEncoder.setPosition(0);
        rightPivotEncoder.setPosition(0);
    }

    public double getLeftPivotEncoderPos() {
        return leftPivotEncoder.getPosition();
    }

    public double getRightPivotEncoderPos() {
        return rightPivotEncoder.getPosition();
    }

    public void periodic() {
        SmartDashboard.putNumber("Left Pivot Encoder", leftPivotEncoder.getPosition());
    }


    public Command SetToTarget(double target) {
        return runOnce(() -> {
            pivotPIDController.setGoal(target);
        })
            .andThen(
                run(
                    () -> {
                        appliedOutput = pivotPIDController.calculate(leftPivotEncoder.getPosition()) 
                        + feedforward.calculate(pivotPIDController.getSetpoint().velocity);
                        leftPivotMotor.setVoltage(appliedOutput);
                    }
                ));
    }
}