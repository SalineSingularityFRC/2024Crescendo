package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StartShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//Named Command For Auton Shooting
public class Shooter extends SequentialCommandGroup {
   public Shooter(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    addCommands(
        new StartShootCommand(shooter),
        new StartIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.stopShooting()
    );
   }
}
