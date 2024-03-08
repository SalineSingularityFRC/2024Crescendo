package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

// For moving the intake back during shooting
public class StartIntakeCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private double initalPosition;

    public StartIntakeCommand(IntakeSubsystem intake) {
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
    }

    public void initialize(){
        this.initalPosition = intakeSubsystem.intakeMotor.getPosition().getValue();
        SmartDashboard.putBoolean("Intake Command", false);
    }

    public void execute() {
        intakeSubsystem.setIntakeSpeed(Constants.Speed.INTAKE * 10);
    }

    public boolean isFinished() {
        double pos = intakeSubsystem.intakeMotor.getPosition().getValue();
        if(pos - initalPosition >= 5){
            SmartDashboard.putBoolean("Intake Command", true);
        }
        return (pos - initalPosition >= 10);
    }
}