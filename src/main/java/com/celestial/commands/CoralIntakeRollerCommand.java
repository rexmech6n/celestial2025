package com.celestial.commands;

import com.celestial.subsystems.CoralIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeRollerCommand extends Command {
    private CoralIntakeSubsystem coralIntakeSubsystem;

    private double speed;

    private boolean previousIsCoralDetected;

    private boolean stop = false;
    private boolean reverse = true;

    private Timer timer = new Timer();

    public CoralIntakeRollerCommand(CoralIntakeSubsystem coralIntakeSubsystem, double speed) {
        this.coralIntakeSubsystem = coralIntakeSubsystem;
        this.speed = speed;
        previousIsCoralDetected = coralIntakeSubsystem.getIsCoralDetected();
        addRequirements(coralIntakeSubsystem);
    }

    @Override
    public void initialize() {
        previousIsCoralDetected = coralIntakeSubsystem.getIsCoralDetected();
        this.stop = false;
        this.reverse = true;
        timer.reset();
    }

    @Override
    public void execute() {
        boolean detected = coralIntakeSubsystem.getIsCoralDetected();

        if(previousIsCoralDetected && !detected) {
            stop = true;
        }

        double power = 0;

        if(stop) {
            if(reverse) {
                timer.start();
                power = -0.15;
            }
            else power = 0.0;
        } else power = speed;

        coralIntakeSubsystem.setSpeed(power);

        previousIsCoralDetected = detected;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.05);
    }

    @Override
    public void end(boolean interrupted) {
        coralIntakeSubsystem.setSpeed(0);
        timer.stop();
    }
}
