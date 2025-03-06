package com.celestial.subsystems;

import com.celestial.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.util.EnumSet;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMasterMotor;
    private final SparkMax elevatorSlaveMotor;
    private final RelativeEncoder elevatorMasterEncoder;

    private ProfiledPIDController elevatorPidController;
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(500, 30);

    private double desiredPosition = 0;

    private BooleanSupplier safetyDependencySupplier;


    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoubleTopic pTopic = inst.getDoubleTopic("el-P");
    DoubleTopic iTopic = inst.getDoubleTopic("el-I");
    DoubleTopic dTopic = inst.getDoubleTopic("el-D");
    DoubleTopic setpointTopic = inst.getDoubleTopic("el-setpoint");

    DoubleSubscriber pSubscriber = pTopic.subscribe(0);
    DoubleSubscriber iSubscriber = iTopic.subscribe(0);
    DoubleSubscriber dSubscriber = dTopic.subscribe(0);
    DoubleSubscriber setpointSubscriber = setpointTopic.subscribe(0);


    public ElevatorSubsystem() {
        elevatorMasterMotor = new SparkMax(Constants.ElevatorConstants.kMasterMotorPort, SparkLowLevel.MotorType.kBrushless);
        elevatorSlaveMotor = new SparkMax(Constants.ElevatorConstants.kSlaveMotorPort, SparkLowLevel.MotorType.kBrushless);

        elevatorPidController = new ProfiledPIDController(
                Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD,
                constraints
        );

        elevatorMasterEncoder = elevatorMasterMotor.getEncoder();

        SparkMaxConfig elevatorMasterMotorConfig = new SparkMaxConfig();
        elevatorMasterMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        elevatorMasterMotorConfig.inverted(true);
        elevatorMasterMotorConfig.encoder
                .positionConversionFactor(Constants.ElevatorConstants.kEncoderRot2Meter)
                .velocityConversionFactor(Constants.ElevatorConstants.kEncoderRPM2MeterPerSecond);

        elevatorMasterMotor.configure(elevatorMasterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig elevatorSlaveMotorConfig = new SparkMaxConfig();
        elevatorSlaveMotorConfig.inverted(true);
        elevatorSlaveMotorConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
        elevatorSlaveMotorConfig.encoder
                .positionConversionFactor(Constants.ElevatorConstants.kEncoderRot2Meter)
                .velocityConversionFactor(Constants.ElevatorConstants.kEncoderRPM2MeterPerSecond);

        elevatorSlaveMotor.configure(elevatorSlaveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        resetElevatorEncoder();


        pTopic.publish().set(Constants.ElevatorConstants.kP);
        iTopic.publish().set(Constants.ElevatorConstants.kI);
        dTopic.publish().set(Constants.ElevatorConstants.kD);
        setpointTopic.publish().set(0);

        inst.addListener(
                pSubscriber,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    System.out.println("P Change");
                    elevatorPidController = new ProfiledPIDController(pSubscriber.get(), iSubscriber.get(), dSubscriber.get(), constraints);
                });

        inst.addListener(
                iSubscriber,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    System.out.println("I Change");
                    elevatorPidController = new ProfiledPIDController(pSubscriber.get(), iSubscriber.get(), dSubscriber.get(), constraints);
                });

        inst.addListener(
                dSubscriber,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    System.out.println("D Change");
                    elevatorPidController = new ProfiledPIDController(pSubscriber.get(), iSubscriber.get(), dSubscriber.get(), constraints);
                });

        inst.addListener(
                setpointSubscriber,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    System.out.println("Setpoint Change");
                    desiredPosition = setpointSubscriber.get();
                });

    }

    @Override
    public void periodic() {
        boolean isSafe = safetyDependencySupplier == null || safetyDependencySupplier.getAsBoolean();

        double height = getHeightCentimeters();

        if(height <= 75 && desiredPosition >= 0 && desiredPosition <= 75 && isSafe) {
            elevatorMasterMotor.set(elevatorPidController.calculate(getHeightCentimeters(), desiredPosition));
            elevatorSlaveMotor.set(elevatorPidController.calculate(getHeightCentimeters(), desiredPosition));
        }

        SmartDashboard.putNumber("Motor Amps", elevatorMasterMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Encoder", elevatorMasterEncoder.getPosition());
        SmartDashboard.putNumber("Elevator Height", getHeightCentimeters());
        SmartDashboard.putNumber("Desired Elevator Height", desiredPosition);
    }

    public boolean getAtDesiredPosition() {
        return Math.abs(getHeightCentimeters() - desiredPosition) < 0.5;
    }

    public double getHeightCentimeters()
    {
        return (elevatorMasterEncoder.getPosition() / Constants.ElevatorConstants.kElevatorGearing) *
                (2 * Math.PI * Constants.ElevatorConstants.kElevatorDrumRadius);
    }

    public void setDesiredPosition(double position) {
        desiredPosition = position;
    }

    public void resetElevatorEncoder() {
        elevatorMasterEncoder.setPosition(0);
    }

    public void resetPID() {
        elevatorPidController.reset(getHeightCentimeters());
    }

    public void setSafetyDependencySupplier(BooleanSupplier safetyDependencySupplier) {
        this.safetyDependencySupplier = safetyDependencySupplier;
    }

    public Command moveElevatorCommand(double position) {
        return parallel(
                runOnce(() -> {
                    setDesiredPosition(position);
                }),
                waitUntil(this::getAtDesiredPosition)
        );
    }
}
