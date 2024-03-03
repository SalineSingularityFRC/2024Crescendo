package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TeleopPreShootCommand extends SequentialCommandGroup {
   public TeleopPreShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {

    addCommands(
        shooter.setShooterBrake(),
        new ReverseIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.setShooterCoast()
    );
   }
}
