package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonIntakeCommand extends SequentialCommandGroup {
   public AutonIntakeCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    addCommands(
        shooter.setShooterBrake(),
        arm.pickupTarget(),
        intake.startIntake(),
        shooter.setShooterCoast()
    );
   }
}