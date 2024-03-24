package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class IntakeParallelCommand extends ParallelCommandGroup {
   public IntakeParallelCommand(ShooterSubsystem shooter, IntakeSubsystem intake, double speed) {

    addCommands(
        intake.startIntake(),
        shooter.reverseShooter(speed)
    );
   }
}
