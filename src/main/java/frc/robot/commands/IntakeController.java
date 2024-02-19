package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

// For moving the intake back during shooting
public class IntakeController extends Command {
    private IntakeSubsystem intakeSubsystem;

    public IntakeController(IntakeSubsystem intake) {
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
    }

    public void execute() {
        intakeSubsystem.setIntakeSpeed(-Constants.Speed.INTAKE/10);
    }

    public boolean isFinished() {
        return (intakeSubsystem.getIntakeSpeed() == -Constants.Speed.INTAKE/10);
    }
}