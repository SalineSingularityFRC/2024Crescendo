package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeParallelCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeBarf extends ParallelCommandGroup {
   public IntakeBarf(IntakeSubsystem intake, ShooterSubsystem shooter) {

    addCommands(
        intake.startIntake(),
        shooter.reverseShooter(-10)
    );
   }
}