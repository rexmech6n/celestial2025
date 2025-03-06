// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.celestial;

import com.celestial.Constants.OperatorConstants;
import com.celestial.commands.CoralIntakeRollerCommand;
import com.celestial.commands.AlgaeIntakeRollerCommand;
import com.celestial.commands.SwerveJoystickCommand;
import com.celestial.subsystems.AlgaeIntakeSubsystem;
import com.celestial.subsystems.CoralIntakeSubsystem;
import com.celestial.subsystems.ElevatorSubsystem;
import com.celestial.subsystems.SwerveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CoralIntakeSubsystem coralIntakeSubsystem = new CoralIntakeSubsystem();
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

    private final CommandPS4Controller commandController =
            new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final PS4Controller controller = new PS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer()
    {
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        elevatorSubsystem.setSafetyDependencySupplier(
                () -> !coralIntakeSubsystem.getIsCoralDetected()
        );

        configureBindings();
    }


    private int currentHeightIndex = 0;

    private double getNextElevatorHeight(int indexIncrement) {
        double[] heights = {1.0, 25.0, 50.0, 70.0};
        int newIndex = indexIncrement + currentHeightIndex;
        System.out.println(newIndex);
        if(newIndex < 0) {
            currentHeightIndex = 0;
           return heights[0];
        }
        if(newIndex >= heights.length) {
            currentHeightIndex = heights.length -1;
            return heights[heights.length - 1];
        }

        currentHeightIndex = newIndex;
        return heights[newIndex];
    }


    private boolean ejection = true;
    private boolean nextEjection = false;

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(
                swerveSubsystem,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> controller.getRightX(),
                () -> !controller.getL2Button()));

        commandController.L1().onTrue(
                elevatorSubsystem.moveElevatorCommand(10.0)
        );

        commandController.povUp().onTrue(
                elevatorSubsystem.moveElevatorCommand(74.5)
        );

        commandController.povDown().onTrue(
                elevatorSubsystem.moveElevatorCommand(0)
        );

        commandController.povLeft().onTrue(
                elevatorSubsystem.moveElevatorCommand(50.0)
        );

        commandController.povRight().onTrue(
                elevatorSubsystem.moveElevatorCommand(25.0)
        );

        commandController.R1().whileTrue(
                new CoralIntakeRollerCommand(coralIntakeSubsystem, 0.4)
        );

        commandController.cross().onTrue(
                new AlgaeIntakeRollerCommand(algaeIntakeSubsystem, () -> 0.2, null)
        );

        commandController.triangle().onTrue(
                new AlgaeIntakeRollerCommand(algaeIntakeSubsystem, () -> -0.4, () -> 0.2)
        );
    }


    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
