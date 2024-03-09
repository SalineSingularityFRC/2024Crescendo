package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.StartShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ShootCommand extends SequentialCommandGroup {
   public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    addCommands(
        shooter.setShooterCoast(),
        new StartShootCommand(shooter),
        new FinalShootCommand(intake, shooter), //Start's the Intake
        intake.stopIntaking()
    );
   }
}
