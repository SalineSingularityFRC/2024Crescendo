package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

public class getToTag extends SequentialCommandGroup{
    public getToTag(SwerveSubsystem drive) {

    addCommands(
        drive.alignToTagCommand(),
        drive.driveToTagCommand(6),
        drive.alignToTagCommand()
    );
   }
}
