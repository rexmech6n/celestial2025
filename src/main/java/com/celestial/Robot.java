package com.celestial;

import com.celestial.auto.AutoAlign;
import com.celestial.subsystems.ElevatorSubsystem;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot
{
    private Command autonomousCommand;
    
    private final RobotContainer robotContainer;

    public Robot()
    {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        super.robotInit();
        AutoAlign.INSTANCE.init();
    }

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
        AutoAlign.INSTANCE.update();
    }

    @Override
    public void disabledInit() {}
    
    
    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit()
    {
        robotContainer.elevatorSubsystem.resetPID();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
        {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        CommandScheduler.getInstance().run();
        //AutoAlign.INSTANCE.actuate();
    }

    @Override
    public void autonomousExit() {
        super.autonomousExit();
        //AutoAlign.INSTANCE.deactuate();
    }

    @Override
    public void teleopInit()
    {
        robotContainer.elevatorSubsystem.resetPID();
        if (autonomousCommand != null)
        {
            autonomousCommand.cancel();
        }
    }
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}
    
    
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
    
    
    /** This method is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    
    /** This method is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
