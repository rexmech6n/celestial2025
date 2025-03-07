package com.celestial.commands;

import com.celestial.Constants;
import com.celestial.auto.AutoAlign;
import com.celestial.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignCommand extends Command {
    private SwerveSubsystem swerveSubsystem;

    public AutoAlignCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        AutoAlign.INSTANCE.arm();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = AutoAlign.INSTANCE.generateChassisSpeeds();
        SwerveModuleState[] states = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        swerveSubsystem.setModuleStates(states);
    }

    @Override
    public void end(boolean interrupted) {
        SwerveModuleState[] states = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
        swerveSubsystem.setModuleStates(states);
        AutoAlign.INSTANCE.disarm();
    }

    @Override
    public boolean isFinished() {
        return AutoAlign.INSTANCE.isAdjustmentDone();
    }
}
