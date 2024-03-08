package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonShooterCommand extends SequentialCommandGroup {
   public AutonShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    addCommands(
        new StartShootCommand(shooter),
        new StartIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.stopShooting()
    );
   }
}
