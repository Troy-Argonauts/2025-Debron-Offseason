package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class CrossStart extends SequentialCommandGroup{

    public CrossStart(){
        super(
            new DriveX(-0.25, Robot.getDrivetrain()).withTimeout(2), //-1.25
            
            new DriveX(0, Robot.getDrivetrain()).withTimeout(0.5)
        );
    }
    
}
