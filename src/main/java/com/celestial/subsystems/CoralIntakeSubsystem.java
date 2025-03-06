package com.celestial.subsystems;

import com.celestial.Constants;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final DigitalInput coralSensor = new DigitalInput(0);

    private final SparkMax intakeMotor = new SparkMax(Constants.CoralIntakeConstants.kMotorPort, SparkMax.MotorType.kBrushless);

    public CoralIntakeSubsystem() {
        SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig.inverted(true);
        intakeMotor.configure(intakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public boolean getIsCoralDetected() {
        SmartDashboard.putBoolean("Coral Detect", coralSensor.get());
        return !coralSensor.get();
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }
}
