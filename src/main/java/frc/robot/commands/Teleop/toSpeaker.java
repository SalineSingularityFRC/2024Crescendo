package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Limelight;

public class toSpeaker extends SequentialCommandGroup{
    public toSpeaker(SwerveSubsystem drive, Limelight lime) {

    addCommands(
        drive.alignAndDriveToTagCommand(lime),
        drive.alignToTagCommand(lime),
        drive.xMode()
    );
   }
}
