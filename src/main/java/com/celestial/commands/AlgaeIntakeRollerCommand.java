package com.celestial.commands;

import com.celestial.subsystems.AlgaeIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class AlgaeIntakeRollerCommand extends Command {
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

    private DoubleSupplier speedSupplier;
    private DoubleSupplier durationSupplier;

    private Timer timer = new Timer();

    public AlgaeIntakeRollerCommand(AlgaeIntakeSubsystem algaeIntakeSubsystem, DoubleSupplier speedSupplier, DoubleSupplier durationSupplier) {
        this.algaeIntakeSubsystem = algaeIntakeSubsystem;
        this.speedSupplier = speedSupplier;
        this.durationSupplier = durationSupplier;
        addRequirements(algaeIntakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        algaeIntakeSubsystem.setSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        if(durationSupplier != null) return timer.hasElapsed(durationSupplier.getAsDouble());
        else return false;
    }

    @Override
    public void end(boolean interrupted) {
        algaeIntakeSubsystem.setSpeed(0);
        timer.stop();
        timer.reset();
    }
}