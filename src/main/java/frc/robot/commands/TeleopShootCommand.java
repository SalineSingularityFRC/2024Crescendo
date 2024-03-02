package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopShootCommand extends SequentialCommandGroup {
   public TeleopShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

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
