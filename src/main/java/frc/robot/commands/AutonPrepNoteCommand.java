package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonPrepNoteCommand extends SequentialCommandGroup {
   public AutonPrepNoteCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {

    addCommands(
        new ReverseIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.startShooting()
    );
   }
}