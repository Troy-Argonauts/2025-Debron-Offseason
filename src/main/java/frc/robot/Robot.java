
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.autonomous.ScoreLV4;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Tongue;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.subsystems.Climber;

/**
 * The VM is configured to automatically run this class, and to call the methods
 * corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private static SwerveSubsystem drivetrain;
    private static Elevator elevator;
    private static Manipulator manipulator;
    private static Climber climber;
    private static RobotContainer robotContainer;
    private static Tongue tongue;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        elevator = new Elevator();
        drivetrain = new SwerveSubsystem();
        manipulator = new Manipulator();
        climber = new Climber();
        tongue = new Tongue();

        robotContainer = new RobotContainer();

        DataLogManager.start("/media/sda1/logs");

        getClimber().resetArmEncoders();

        


        // CameraServer.startAutomaticCapture().setFPS(14);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("operator.getLeftY", RobotContainer.getOperator().getLeftY()*0.3);
    }

    @Override
    public void disabledInit() {
        getDrivetrain().setToZero();
        getElevator().resetTarget();    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }

    /**
     * Gets the Elevator object
     * 
     * @return Elevator object
     */
    public static Elevator getElevator() {
        if (elevator == null) {
            elevator = new Elevator();
        }
        return elevator;
    }

    /**
     * Gets the SwerveSubsystem object
     * 
     * @return SwerveSubsystem object
     */
    public static SwerveSubsystem getDrivetrain() {
        if (drivetrain == null) {
            drivetrain = new SwerveSubsystem();
        }
        return drivetrain;
    }

    /**
     * Gets the Manipulator object
     * 
     * @return Manipulator object
     */
    public static Manipulator getManipulator() {
        if (manipulator == null)
            manipulator = new Manipulator();
        return manipulator;
    }

    /**
     * Gets the RobotContainer object
     * 
     * @return RobotContainer object
     */
    public static RobotContainer getRobotContainer() {
        if (robotContainer == null)
        robotContainer = new RobotContainer();
        return robotContainer;
    }

  public static Climber getClimber() {
    if (climber == null) climber = new Climber();
    return climber;
}

    public static Tongue getTongue() {
        if (tongue == null) tongue = new Tongue();
        return tongue;
    }
}
