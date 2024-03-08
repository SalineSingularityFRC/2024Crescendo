package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonMiddlePreShooter extends SequentialCommandGroup{
    public AutonMiddlePreShooter(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    addCommands(
        arm.autonShootTarget(Constants.Position.MainArm.AUTONLMIDDLESHOOT),
        new ReverseIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.autonStartUpShooter()
    );
   }
}
