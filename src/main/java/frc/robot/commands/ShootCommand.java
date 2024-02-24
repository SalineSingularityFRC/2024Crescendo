package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
   public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {

    addCommands(
        new ReverseIntakeCommand(intake),
        intake.stopIntaking(),
        new StartShootCommand(shooter), 
        new StartIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.stopShooting()
    );
   }
}
