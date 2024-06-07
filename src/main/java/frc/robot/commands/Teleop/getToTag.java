package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Limelight;

public class getToTag extends SequentialCommandGroup{
    public getToTag(SwerveSubsystem drive, Limelight lime) {

    addCommands(
        drive.alignAndDriveToTagCommand(6, lime),
        drive.alignToTagCommand(lime)
    );
   }
}
