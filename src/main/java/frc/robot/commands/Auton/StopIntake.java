package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StartShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//Named Command For Auton Shooting
public class StopIntake extends SequentialCommandGroup {
   public StopIntake(ShooterSubsystem shooter, IntakeSubsystem intake) {

    addCommands(
        intake.autonStopIntaking(shooter),
        intake.setBrake()
    );
   }
}
