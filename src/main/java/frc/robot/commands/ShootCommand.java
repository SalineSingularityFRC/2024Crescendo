package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootCommand extends SequentialCommandGroup {
   public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm, SwerveSubsystem s) {

    addCommands(
        s.setBrakeModeCommand(),
        new StartShootCommand(shooter), 
        new StartIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.stopShooting(),
        s.setCoastModeCommand()
    );
   }
}
