package com.celestial.subsystems;

import com.celestial.Constants;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(Constants.AlgaeIntakeConstants.kMotorPort, SparkMax.MotorType.kBrushless);

    public AlgaeIntakeSubsystem() {
        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig.inverted(false);
        intakeMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        intakeMotor.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

}
