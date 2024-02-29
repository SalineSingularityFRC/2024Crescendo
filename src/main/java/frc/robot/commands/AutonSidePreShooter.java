package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonSidePreShooter extends SequentialCommandGroup{
    public AutonSidePreShooter(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    addCommands(
        new AutonPreShootCommand(shooter, intake, arm, Constants.Position.MainArm.AUTONSIDESHOOT)
    );
   }
}
