package com.celestial.subsystems;

import com.celestial.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase
{

    private SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftRotationMotorPort,
            DriveConstants.kFrontLeftDriveReversed,
            DriveConstants.kFrontLeftRotationReversed,
            DriveConstants.kFrontLeftAbsoluteEncoderPort,
            DriveConstants.kFrontLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftAbsoluteEncoderReversed);

    private SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightRotationMotorPort,
            DriveConstants.kFrontRightDriveReversed,
            DriveConstants.kFrontRightRotationReversed,
            DriveConstants.kFrontRightAbsoluteEncoderPort,
            DriveConstants.kFrontRightAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightAbsoluteEncoderReversed);

    private SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftRotationMotorPort,
            DriveConstants.kBackLeftDriveReversed,
            DriveConstants.kBackLeftRotationReversed,
            DriveConstants.kBackLeftAbsoluteEncoderPort,
            DriveConstants.kBackLeftAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftAbsoluteEncoderReversed);

    private SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightRotationMotorPort,
            DriveConstants.kBackRightDriveReversed,
            DriveConstants.kBackRightRotationReversed,
            DriveConstants.kBackRightAbsoluteEncoderPort,
            DriveConstants.kBackRightAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getSwerveModulePositions());

    private Field2d field = new Field2d();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    StructArrayPublisher<SwerveModuleState> publisher = inst.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();


    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                () -> DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleStates()),
                this::driveRelative,
                new PPHolonomicDriveController(
                        new PIDConstants(25.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {


                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );


    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(gyro.getRotation2d(), getSwerveModulePositions(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePositions());
        field.setRobotPose(odometer.getPoseMeters());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Front Left Absolute", Units.radiansToDegrees(frontLeft.getAbsoluteEncoderRad()));
        SmartDashboard.putNumber("Front Left Drive", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("Front Right Drive", frontRight.getDrivePosition());
        SmartDashboard.putNumber("Back Left Drive", backLeft.getDrivePosition());
        SmartDashboard.putNumber("Back Right Drive", backRight.getDrivePosition());

        SmartDashboard.putNumber("Back Left Absolute", Units.radiansToDegrees(backLeft.getAbsoluteEncoderRad()));

        SmartDashboard.putNumber("Front Right Absolute", Units.radiansToDegrees(frontRight.getAbsoluteEncoderRad()));

        SmartDashboard.putNumber("Back Right Absolute", Units.radiansToDegrees(backRight.getAbsoluteEncoderRad()));
        publisher.set(getSwerveModuleStates());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void driveRelative(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);

        setModuleStates(setpointStates);
    }
}
