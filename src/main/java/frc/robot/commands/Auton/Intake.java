package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeParallelCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Intake extends SequentialCommandGroup {
   public Intake(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm) {

    addCommands(
        intake.setCoast(),
        arm.pickupTarget(),
        new IntakeParallelCommand(shooter, intake, Constants.Speed.REVERSESHOOTER)
    );
   }
}