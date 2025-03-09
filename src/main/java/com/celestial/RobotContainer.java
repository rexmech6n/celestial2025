// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.celestial;

import com.celestial.Constants.OperatorConstants;
import com.celestial.commands.AutoAlignCommand;
import com.celestial.commands.CoralIntakeRollerCommand;
import com.celestial.commands.AlgaeIntakeRollerCommand;
import com.celestial.commands.SwerveJoystickCommand;
import com.celestial.subsystems.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final CoralIntakeSubsystem coralIntakeSubsystem = new CoralIntakeSubsystem();
    private final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final CommandPS4Controller commandController =
            new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final PS4Controller controller = new PS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer()
    {
        elevatorSubsystem.setSafetyDependencySupplier(
                () -> !coralIntakeSubsystem.getIsCoralDetected()
        );

        NamedCommands.registerCommand("ElevatorAlgae", elevatorSubsystem.moveElevatorCommand(() -> 34.0));
        NamedCommands.registerCommand("ElevatorCoral", elevatorSubsystem.moveElevatorCommand(() -> 45.0));
        NamedCommands.registerCommand("ElevatorHome", elevatorSubsystem.moveElevatorCommand(() -> 0.0));
        NamedCommands.registerCommand("GoForwardSlowly", Commands.parallel(
                Commands.run(
                        () -> {
                            ChassisSpeeds speeds = new ChassisSpeeds(0.2, 0, 0);
                            swerveSubsystem.driveRelative(speeds);
                        }
                ).withTimeout(1.5)
        ).finallyDo(
                () -> {
                    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
                    swerveSubsystem.driveRelative(speeds);
                }
        ));


        NamedCommands.registerCommand("GoBackFastly", Commands.parallel(
                Commands.run(
                        () -> {
                            ChassisSpeeds speeds = new ChassisSpeeds(-0.4, 0, 0);
                            swerveSubsystem.driveRelative(speeds);
                        }
                ).withTimeout(1)
        ).finallyDo(
                () -> {
                    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
                    swerveSubsystem.driveRelative(speeds);
                }
        ));

        NamedCommands.registerCommand("TakeAlgae", algaeIntakeSubsystem.setSpeedCommand(0.2));
        NamedCommands.registerCommand("DropAlgae", algaeIntakeSubsystem.setSpeedCommand(0.0));
        NamedCommands.registerCommand("DropCoral", Commands.parallel(
                Commands.runOnce(() -> coralIntakeSubsystem.setSpeed(0.4)),
                Commands.waitSeconds(1.5)
        ).finallyDo(() -> coralIntakeSubsystem.setSpeed(0)));
        NamedCommands.registerCommand("AlignToReef",  new AutoAlignCommand(swerveSubsystem));


        configureBindings();
        SmartDashboard.putString("Elevator Stage", "Home");

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }


    private int currentHeightIndex = 0;

    private double getNextElevatorHeight(int indexIncrement) {
        double[] heights = {0.0, 5.0, 12.0, 23.0, 34, 43, 54, 74.5};
        int newIndex = indexIncrement + currentHeightIndex;
        System.out.println(newIndex);

        if(newIndex < 0) {
            currentHeightIndex = 0;
            displayElevatorStage(0);
           return heights[0];
        }
        if(newIndex >= heights.length) {
            currentHeightIndex = heights.length -1;
            displayElevatorStage(heights.length - 1);
            return heights[heights.length - 1];
        }

        displayElevatorStage(newIndex);

        currentHeightIndex = newIndex;
        return heights[newIndex];
    }

    private void displayElevatorStage(int newIndex) {
        String stage = newIndex == 0 ? "Home" :
                newIndex == 1 ? "Algae Put 1" :
                newIndex == 2 ? "L1" :
                newIndex == 3 ? "L2" :
                newIndex == 4 ? "Algae Put 2" :
                newIndex == 5 ? "L3" :
                newIndex == 6 ? "Algae Put 3"
                        : "L4";

        SmartDashboard.putString("Elevator Stage", stage);
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


        commandController.povUp().onTrue(
                elevatorSubsystem.moveElevatorCommand(() -> 74.5) // L4
        );

        commandController.povDown().onTrue(
                elevatorSubsystem.moveElevatorCommand(() -> 0.0) // L1
        );

        commandController.povLeft().onTrue(
                elevatorSubsystem.moveElevatorCommand(() -> 23) // L2
        );

        commandController.povRight().onTrue(
                elevatorSubsystem.moveElevatorCommand(() -> 45) // L3
        );

        commandController.R1().whileTrue(
                new CoralIntakeRollerCommand(coralIntakeSubsystem, 0.4)
        );

        commandController.L1().whileTrue(
                coralIntakeSubsystem.setSpeedCommand(-0.2)
        );

        /*commandController.R2().whileTrue(
                climberSubsystem.controlClimberCommand(-0.3)
        );

        commandController.L2().whileTrue(
                climberSubsystem.controlClimberCommand(0.3)
        );*/

        commandController.cross().onTrue(
                new AlgaeIntakeRollerCommand(algaeIntakeSubsystem, () -> 0.2, null)
        );

        commandController.triangle().onTrue(
                new AlgaeIntakeRollerCommand(algaeIntakeSubsystem, () -> -0.5, () -> 0.2)
        );

        commandController.square().onTrue(
                elevatorSubsystem.moveElevatorCommand(() -> 36)
        );

        commandController.circle().onTrue(
                elevatorSubsystem.moveElevatorCommand(() -> 54)
        );

        commandController.options().onTrue(
                elevatorSubsystem.moveElevatorCommand(() -> 5.0)
        );

        commandController.share().whileTrue(
                new AutoAlignCommand(swerveSubsystem)
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
