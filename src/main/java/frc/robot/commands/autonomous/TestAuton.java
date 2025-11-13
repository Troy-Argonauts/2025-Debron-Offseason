package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Manipulator.ManipulatorStates;

public class TestAuton extends SequentialCommandGroup {
    public TestAuton(){
        super(

            //new DriveX(-1.25, Robot.getDrivetrain()).withTimeout(1.2), //-1.25
            new DriveX(-0.25, Robot.getDrivetrain()).withTimeout(2.2), //-1.25

            new DriveX(0, Robot.getDrivetrain()).withTimeout(0.5),

           // new WaitCommand(0.5),
            
            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.LV4), Robot.getElevator()),

            new WaitUntilCommand(() -> Robot.getElevator().isPIDFinished()),

            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OUTAKE), Robot.getManipulator()),

            new WaitCommand(1),

            new InstantCommand(() -> Robot.getManipulator().setManipState(ManipulatorStates.OFF), Robot.getManipulator()),

            new InstantCommand(() -> Robot.getElevator().setDesiredState(ElevatorStates.HOME), Robot.getElevator())

        );
        // Evan is soo not tuff, ved is soo much more tuff
    }
}