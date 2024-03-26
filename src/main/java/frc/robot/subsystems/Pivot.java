// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private CANSparkMax PivotMotor;

    private RelativeEncoder PivotMotorEncoder;


    public Pivot() {

        this.PivotMotor = new CANSparkMax(3, MotorType.kBrushless);

        this.PivotMotorEncoder = this.PivotMotor.getEncoder();
    
        this.PivotMotor.restoreFactoryDefaults();

        PivotMotor.setInverted(false);
        
        this.PivotMotor.burnFlash();

        
        // pidController = frontLeftDriveMotor.getPIDController();
        // pidController.setP(kP);
        // pidController.setI(kI);
        // pidController.setD(kD);
        // pidController.setIZone(kIz);
        // pidController.setFF(kFF);
        // pidController.setOutputRange(kMinOutput, kMaxOutput);
    
        this.PivotMotorEncoder.setPosition(0);
        SmartDashboard.putNumber("get position", this.PivotMotorEncoder.getPosition());


    }

    /**
     * calls stopMotor method within {@link edu.wpi.first.wpilibj.drive.DifferentialDrive}
     * to stop motors
     */
    public void PivotStop() {
        this.PivotMotor.set(0);
    }

    public void resetEncoders() {
        PivotMotorEncoder.setPosition(0);
    }

    public double getPivotMotorEncoderPos() {
        return PivotMotorEncoder.getPosition();
    }

    /**
     * scales value ranging from -1 to 1 to 0 to 1
     * @param rawValue raw value from joystick; ranging from -1 to 1
     * @return scaled value; ranging from 0 to 1
     */
    public double adjustedSpeed(double rawValue){
        if (-rawValue <= 0) {
            return 0.5 - (Math.abs(-rawValue) / 2);
        } 
        return (-rawValue / 2) + 0.5;
    }
}