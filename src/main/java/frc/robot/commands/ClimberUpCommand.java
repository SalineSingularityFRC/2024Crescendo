package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCommand extends SequentialCommandGroup {
   public ClimberUpCommand(ClimberSubsystem climber, ArmSubsystem arm) {

    addCommands(
        arm.climberTarget(),
        climber.moveClimberUp()
    );
   }
}
