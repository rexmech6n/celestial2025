// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.celestial;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static final class CoralIntakeConstants {
        public static final int kMotorPort = 12;
    }

    public static final class AlgaeIntakeConstants {
        public static final int kMotorPort = 11;
    }

    public static final class ElevatorConstants {
        public static final int kMasterMotorPort = 9;
        public static final int kSlaveMotorPort = 10;
        public static final double kP = 0.15;
        public static final double kI = 0.0;
        public static final double kD = 0;
        public static final int kEncoderRot2Meter = 1;
        public static final int kEncoderRPM2MeterPerSecond = 1;
        public static final double kElevatorGearing = 21.064123994;
        public static final double kElevatorDrumRadius = 1.5288; // centimeters
    }

    public static final class ModuleConstants {
        public static final double kDriveMotorGearRatio = 1 / 5.54;
        public static final double kRotationMotorGearRatio = 1 / 25.0;
        public static final double kWheelDiamater = Units.inchesToMeters(4);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiamater;
        public static final double kRotationEncoderRot2Rad = kRotationMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kRotationEncoderRPM2RadPerSec = kRotationEncoderRot2Rad / 60;
        public static final double kPRotation = 0.6;
        public static final double kDRotation = 0.008;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.57;

        // Distance between right and left wheels
        public static final double kWheelBase = 0.57;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 6;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 2;

        public static final int kFrontLeftRotationMotorPort = 5;
        public static final int kBackLeftRotationMotorPort = 7;
        public static final int kFrontRightRotationMotorPort = 3;
        public static final int kBackRightRotationMotorPort = 1;

        public static final boolean kFrontLeftRotationReversed = false;
        public static final boolean kBackLeftRotationReversed = false;
        public static final boolean kFrontRightRotationReversed = false;
        public static final boolean kBackRightRotationReversed = false;

        public static final boolean kFrontLeftDriveReversed = false;
        public static final boolean kBackLeftDriveReversed = true;
        public static final boolean kFrontRightDriveReversed = false;
        public static final boolean kBackRightDriveReversed = false;

        public static final int kFrontLeftAbsoluteEncoderPort = 2;
        public static final int kBackLeftAbsoluteEncoderPort = 3;
        public static final int kFrontRightAbsoluteEncoderPort = 1;
        public static final int kBackRightAbsoluteEncoderPort = 0;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightAbsoluteEncoderReversed = false;
        public static final boolean kBackRightAbsoluteEncoderReversed = false;

        public static final double kFrontLeftAbsoluteEncoderOffsetRad = Units.degreesToRadians(53);
        public static final double kBackLeftAbsoluteEncoderOffsetRad = Units.degreesToRadians(311);
        public static final double kFrontRightAbsoluteEncoderOffsetRad = Units.degreesToRadians(199.2);
        public static final double kBackRightAbsoluteEncoderOffsetRad = Units.degreesToRadians(355);

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5.57;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }
}
