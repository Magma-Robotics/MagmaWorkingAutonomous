// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    public static class Control {

        public static class ControllerPort {
            public static final int kDRIVER = 0;
            public static final int kPARTNER = 1; 
        }
    }


    public static class Subsystems {
        public static class Intake{
            public static final int kIntakeId = 12;
            public static final double kPOWER = 0.7;
        }
        public static class Shooter{
            public static final int kLeftShooterId = 7;//10;
            public static final int kRightShooterId = 8;//11;
            public static final int kPushMotorId = 17;
            public static final double kPOWER = 0.7;
            public static final double kPOWERWEAKER = 0.5;

            public static final double kLeftS = 0.23643;
            public static final double kLeftV = 0.38942;
            public static final double kLeftA = 0.046395;

            public static final double kRightS = 0.20658;
            public static final double kRightV = 0.38488;
            public static final double kRightA = 0.041661;

            public static final double kHome = 0;
            public static final double kSpeaker = 0.7;
            public static final double kAmp = 1;

        }
        public static class Lift{
            public static final int kLeftLiftId = 13;
            public static final int kRightLiftId = 14;
            public static final double kPOWER = 1;
        }
        public static class DriveTrain { 
            public static final int kFrontLeftId = 1;
            public static final int kRearLeftId = 2;
            public static final int kFrontRightId = 3;
            public static final int kRearRightId = 4;
            
            public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
            public static final double kTrackWidthMeters = Units.inchesToMeters(22.5);
            public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

            public static final double kLinearDistanceConversionFactor = 0.12;
            
            public static final double kLeftP = 0.1;
            public static final double kLeftI = 0;
            public static final double kLeftD = 0;
            public static final double kLeftS = 0;
            public static final double kLeftV = 0;
            public static final double kLeftA = 0;
            public static final double kRightP = 0.1;
            public static final double kRightI = 0;
            public static final double kRightD = 0;
            public static final double kRightS = 0;
            public static final double kRightV = 0;
            public static final double kRightA = 0;
        }
        public static class Pivot {
            public static final int kLeftPivotId = 15;
            public static final int kRightPivotId = 16;
            public static final double kLinearDistanceConversionFactor = 0.5;
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
        }
    }


}