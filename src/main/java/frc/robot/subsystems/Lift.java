// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Lift extends SubsystemBase {


    /**
     * an abstract representation of a physical robot arm
     */
    private CANSparkMax leftLift, rightLift;

  
    /**
     * subsystem base object for arm
     */
    public Lift() {
        this.leftLift = new CANSparkMax(Constants.Subsystems.Lift.kLeftLiftId, MotorType.kBrushless);
        this.rightLift = new CANSparkMax(Constants.Subsystems.Lift.kRightLiftId, MotorType.kBrushless);
    }


    /**
     * arm goes up by setting power on the arm motor
     */
    public void LiftUp() {
        this.leftLift.set(Constants.Subsystems.Lift.kPOWER);
        this.rightLift.set(Constants.Subsystems.Lift.kPOWER);
    }


     /**
     * second arm goes down by setting power on the arm motor
     */
    public void LiftDown() {
        this.leftLift.set(-Constants.Subsystems.Lift.kPOWER);
        this.rightLift.set(-Constants.Subsystems.Lift.kPOWER);
    }


    /**
     * calls stopMotor method within {@link edu.wpi.first.wpilibj.drive.DifferentialDrive}
     * to stop second motors
     */
    public void LiftStop() {
        this.leftLift.stopMotor();
        this.rightLift.stopMotor();
    }

    public void LiftUpAuto(double power) {
        this.leftLift.set(power);
        this.rightLift.set(power);
    }


     /**
     * second arm goes down by setting power on the arm motor
     */
    public void LiftDownAuto(double power) {
        this.leftLift.set(-power);
        this.rightLift.set(power);
    }
  
}