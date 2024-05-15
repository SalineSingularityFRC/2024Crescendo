package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PreShooter extends SequentialCommandGroup{
    public PreShooter(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm, double position) {

    addCommands(
        intake.stopIntaking(), //maybe not needed
        arm.autonShootTarget(position),
        new ReverseIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.setShooterCoast(),
        shooter.autonStartUpShooter()
    );
   }
}
