package com.celestial.commands;

import com.celestial.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class ElevatorControlCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    private DoubleSupplier desiredPositionSupplier;
    private double desiredPosition;

    private boolean isSet = false;

    public ElevatorControlCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier desiredPositionSupplier) {
        this.desiredPositionSupplier = desiredPositionSupplier;
        this.elevatorSubsystem = elevatorSubsystem;
        this.desiredPosition = desiredPositionSupplier.getAsDouble();
        addRequirements(elevatorSubsystem);
        System.out.println(desiredPosition);
    }

    @Override
    public void initialize() {
        this.desiredPosition = desiredPositionSupplier.getAsDouble();
    }

    @Override
    public void execute() {
        if(isSet && elevatorSubsystem.getAtDesiredPosition()) return;

        elevatorSubsystem.setDesiredPosition(desiredPosition);
        isSet = true;
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getAtDesiredPosition();
    }

    @Override
    public void end(boolean interrupted) {
        isSet = false;
    }
}
