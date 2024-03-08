package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonPreShootCommand extends SequentialCommandGroup {
   public AutonPreShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm, 
    double position) {

    addCommands(
        arm.autonShootTarget(position),
        new ReverseIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.startShooting()
    );
   }
}
