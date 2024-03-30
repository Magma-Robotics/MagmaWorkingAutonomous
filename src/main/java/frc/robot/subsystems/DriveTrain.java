// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import java.lang.Math;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {


    /**
     * an abstract representation of a physical drive motor
     */

    
    private CANSparkMax frontLeftDrive = new CANSparkMax(Constants.Subsystems.DriveTrain.kFrontLeftId, MotorType.kBrushless);
    private CANSparkMax rearLeftDrive = new CANSparkMax(Constants.Subsystems.DriveTrain.kRearLeftId, MotorType.kBrushless);
    private CANSparkMax frontRightDrive = new CANSparkMax(Constants.Subsystems.DriveTrain.kFrontRightId, MotorType.kBrushless);
    private CANSparkMax rearRightDrive = new CANSparkMax(Constants.Subsystems.DriveTrain.kRearRightId, MotorType.kBrushless);


    private RelativeEncoder leftDriveEncoder = frontLeftDrive.getEncoder();
    private RelativeEncoder rightDriveEncoder = frontRightDrive.getEncoder();
    private DifferentialDrive diffDrive;  

    private AHRS navx = new AHRS(SPI.Port.kMXP);

    private Field2d field;

    private DifferentialDriveKinematics kinematics = Constants.Subsystems.DriveTrain.kDriveKinematics;
    private double wheelRadius = Constants.Subsystems.DriveTrain.kWheelRadiusMeters;

    private PIDController leftWheelsController = new PIDController(
        Constants.Subsystems.DriveTrain.kLeftP, Constants.Subsystems.DriveTrain.kLeftI, Constants.Subsystems.DriveTrain.kLeftD
        ); 
    private PIDController rightWheelsController = new PIDController(
        Constants.Subsystems.DriveTrain.kRightP, Constants.Subsystems.DriveTrain.kRightI, Constants.Subsystems.DriveTrain.kRightD
        ); 

    private SimpleMotorFeedforward leftWheelFeedforward = new SimpleMotorFeedforward(
        Constants.Subsystems.DriveTrain.kLeftS, Constants.Subsystems.DriveTrain.kLeftV, Constants.Subsystems.DriveTrain.kLeftA
        );

    private SimpleMotorFeedforward rightWheelFeedforward = new SimpleMotorFeedforward(
        Constants.Subsystems.DriveTrain.kRightS, Constants.Subsystems.DriveTrain.kRightV, Constants.Subsystems.DriveTrain.kRightA
        );

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0)); 
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
            frontLeftDrive.setVoltage(volts.in(Volts));
            frontRightDrive.setVoltage(volts.in(Volts));
        }, 
        log -> {
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        frontLeftDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(leftDriveEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(leftDriveEncoder.getVelocity(), MetersPerSecond));
            log.motor("drive-right")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            frontRightDrive.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(rightDriveEncoder.getPosition(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(rightDriveEncoder.getVelocity(), MetersPerSecond));
              },
        this));


    public DriveTrain() {
        this.frontLeftDrive.restoreFactoryDefaults();
        this.rearLeftDrive.restoreFactoryDefaults();
        this.frontRightDrive.restoreFactoryDefaults();
        this.rearRightDrive.restoreFactoryDefaults();

        frontLeftDrive.setInverted(false);
        frontRightDrive.setInverted(true);

        rearRightDrive.follow(frontRightDrive);
        rearLeftDrive.follow(frontLeftDrive);

        leftDriveEncoder.setPositionConversionFactor(.039);
        rightDriveEncoder.setPositionConversionFactor(.039);

        leftDriveEncoder.setVelocityConversionFactor(.039 / 60);
        rightDriveEncoder.setVelocityConversionFactor(.039 / 60);

        //this.leftDriveEncoder.setPositionConversionFactor(Constants.Subsystems.DriveTrain.kLinearDistanceConversionFactor);
        //this.rightDriveEncoder.setPositionConversionFactor(Constants.Subsystems.DriveTrain.kLinearDistanceConversionFactor);
        
        this.frontLeftDrive.burnFlash();
        this.rearLeftDrive.burnFlash();
        this.frontRightDrive.burnFlash();
        this.rearRightDrive.burnFlash();

        AutoBuilder.configureRamsete(
            this::getPose, 
            (Pose2d pose) -> {
                this.resetPose(pose);
            }, 
            this::getSpeeds,
            (ChassisSpeeds chassisSpeeds) -> {
                this.driveConsumer(chassisSpeeds);
            }, 
            new ReplanningConfig(), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                }, 
            this);

    
        this.diffDrive = new DifferentialDrive(this.frontLeftDrive, this.frontRightDrive);
        SmartDashboard.putData(diffDrive);
        resetEncoders();
        navx.reset();

        field = new Field2d();
        SmartDashboard.putData("Field", field);

        SmartDashboard.putData("Left Wheel PID", leftWheelsController);
        SmartDashboard.putData("Right Wheel PID", rightWheelsController);
    }

    public void periodic() {
        SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public void driveConsumer(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        frontLeftDrive.setVoltage(leftWheelFeedforward.calculate(wheelSpeeds.leftMetersPerSecond)
            + leftWheelsController.calculate(leftDriveEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond));
        frontRightDrive.setVoltage(rightWheelFeedforward.calculate(wheelSpeeds.rightMetersPerSecond)
            + rightWheelsController.calculate(rightDriveEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond));
    }

    private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        leftDriveEncoder.getVelocity(), rightDriveEncoder.getVelocity()); //rpm in m/s
    private DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(
        leftDriveEncoder.getPosition(), rightDriveEncoder.getPosition()); //encoder ticks in meters
    private ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    public ChassisSpeeds getSpeeds() {
        return chassisSpeeds;
    }

    private final DifferentialDrivePoseEstimator m_PoseEstimator =
        new DifferentialDrivePoseEstimator(
            kinematics, //track width
            navx.getRotation2d(),
            wheelPositions.leftMeters, //encoders
            wheelPositions.rightMeters, 
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    public Pose2d getPose() {
        return m_PoseEstimator.getEstimatedPosition();
    }
    
    public void resetPose(Pose2d pose) {
        m_PoseEstimator.resetPosition(navx.getRotation2d(), wheelPositions, pose);
    }

    public void stop() {
        this.diffDrive.stopMotor();
    }

    public void resetEncoders() {
        leftDriveEncoder.setPosition(0);
        rightDriveEncoder.setPosition(0);
    }

    public double getLeftEncoderPos() {
        return leftDriveEncoder.getPosition();
    }

    public double getRightEncoderPos() {
        return rightDriveEncoder.getPosition();
    }

    public void diffDriveJoystick(double leftJoystick, double rightJoystick) {
        diffDrive.tankDrive(-leftJoystick, -rightJoystick);
    }

    public void diffDrive(double leftPower, double rightPower) {
        diffDrive.tankDrive(leftPower, rightPower);
    }


}