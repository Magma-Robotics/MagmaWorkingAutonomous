// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {


    /**
     * an abstract representation of a physical robot arm
     */
    private CANSparkMax leftShooter, rightShooter, pushMotor;
    private RelativeEncoder leftEncoder = leftShooter.getEncoder();
    private RelativeEncoder rightEncoder = rightShooter.getEncoder();

    private SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(
        Constants.Subsystems.Shooter.kLeftS, Constants.Subsystems.Shooter.kLeftV, Constants.Subsystems.Shooter.kLeftA);

    private SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(
        Constants.Subsystems.Shooter.kRightS, Constants.Subsystems.Shooter.kRightV, Constants.Subsystems.Shooter.kRightA);

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RPM.of(0));

    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                    leftShooter.setVoltage(volts.in(Volts));
                },
                log -> {
                    log.motor("shooter-left")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                leftShooter.get() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(m_angle.mut_replace(leftEncoder.getPosition(), Rotations))
                        .angularVelocity(
                            m_velocity.mut_replace(leftEncoder.getVelocity(), RPM));

                    log.motor("shooter-right")
                        .voltage(
                            m_appliedVoltage.mut_replace(
                                rightShooter.get() * RobotController.getBatteryVoltage(), Volts))
                        .angularPosition(m_angle.mut_replace(rightEncoder.getPosition(), Rotations))
                        .angularVelocity(
                            m_velocity.mut_replace(rightEncoder.getVelocity(), RPM));
                },
                this));

  
    /**
     * subsystem base object for arm
     */
    public Shooter() {
        this.leftShooter = new CANSparkMax(Constants.Subsystems.Shooter.kLeftShooterId, MotorType.kBrushless);
        this.rightShooter = new CANSparkMax(Constants.Subsystems.Shooter.kRightShooterId, MotorType.kBrushless);
        pushMotor = new CANSparkMax(Constants.Subsystems.Shooter.kPushMotorId, MotorType.kBrushless);
        
        leftShooter.setInverted(true);

        leftShooter.burnFlash();
        
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    public Command TargetFeedforward(double leftWheel, double rightWheel) {
        return run(
            () -> {
                leftShooter.setVoltage(leftFeedforward.calculate(leftWheel));
                rightShooter.setVoltage(rightFeedforward.calculate(rightWheel));
            }
        );
    }

    public void stopShooter() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
    }

    public void stopPushMotor() {
        pushMotor.stopMotor();
    }

    public void shooterForward() {
        leftShooter.set(Constants.Subsystems.Shooter.kPOWER);
        rightShooter.set(Constants.Subsystems.Shooter.kPOWER);
    }

    public void shooterBackward() {
        leftShooter.set(-Constants.Subsystems.Shooter.kPOWER);
        rightShooter.set(-Constants.Subsystems.Shooter.kPOWER);
    }

    public void pushMotorForward(double power) {
        pushMotor.set(-power);
    }

    public void pushMotorBackward(double power) {
        pushMotor.set(power);
    }

    public void autoShooterForward(double power) {
        leftShooter.set(power);
        rightShooter.set(power);
    }

    public void autoShooterBackward(double power) {
        leftShooter.set(-power);
        rightShooter.set(-power);
    }
/*
    public void ShooterMotor1Forward() {
        this.ShooterMotor1.set(Constants.Subsystems.Shooter.kPOWER);
    }

    public void ShooterMotor1Backward() {
        this.ShooterMotor1.set(-Constants.Subsystems.Shooter.kPOWER);
    }


    public void ShooterMotor1Stop() {
        this.ShooterMotor1.stopMotor();
    }


    public void ShooterMotor2Forward() {
        this.ShooterMotor2.set(Constants.Subsystems.Shooter.kPOWER);
    }

    public void ShooterMotor2Backward() {
        this.ShooterMotor2.set(-Constants.Subsystems.Shooter.kPOWER);
    }

    public void ShooterMotor2Stop() {
        this.ShooterMotor2.stopMotor();
    }

    public void ShooterMotor1ForwardAuto(double power) {
        this.ShooterMotor1.set(power);
    }

    public void ShooterMotor1BackwardAuto(double power) {
        this.ShooterMotor1.set(-power);
    }

    public void ShooterMotor2ForwardAuto(double power) {
        this.ShooterMotor2.set(power);
    }

    public void ShooterMotor2BackwardAuto(double power) {
        this.ShooterMotor2.set(-power);
    }
    
    public void ShooterMotor1ForwardMid() {
        this.ShooterMotor1.set(0.6);
    }

    public void ShooterMotor2BackwardMid() {
        this.ShooterMotor2.set(-0.6);
    }

    public void ShooterMotor1ForwardWeaker() {
        this.ShooterMotor1.set(0.5);
    }

    public void ShooterMotor2BackwardWeaker() {
        this.ShooterMotor2.set(-0.5);
    }
*/
}