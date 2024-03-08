package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDownCommand extends SequentialCommandGroup {
   public ClimberDownCommand(ClimberSubsystem climber, ArmSubsystem arm) {

    addCommands(
        
        climber.moveClimberDown()
    );
   }
}
