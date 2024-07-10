package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Limelight;

public class toAmp extends SequentialCommandGroup{
    public toAmp(SwerveSubsystem drive, Limelight lime) {

    addCommands(
        drive.alignAndGetPerpendicularToTagCommand(lime)
    );
   }
}
