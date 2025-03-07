package com.celestial.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climberMotor = new SparkMax(13, SparkLowLevel.MotorType.kBrushless);

    public ClimberSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(23);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);

        climberMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        climberMotor.set(speed);
    }

    public Command controlClimberCommand(double speed) {
        return run(() -> {
           setSpeed(speed);
        }).finallyDo(() -> {
            setSpeed(0);
        });
    }
}
