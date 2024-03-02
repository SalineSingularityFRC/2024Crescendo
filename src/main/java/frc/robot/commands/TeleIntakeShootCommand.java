package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// For moving the intake back during shooting
public class TeleIntakeShootCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private double initalPosition;

    public TeleIntakeShootCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
        intakeSubsystem = intake;
        shooterSubsystem = shooter;
        addRequirements(intakeSubsystem, shooter);
    }

    public void initialize(){
        this.initalPosition = intakeSubsystem.intakeMotor.getPosition().getValue();
        SmartDashboard.putBoolean("Intake Command", false);
    }

    public void execute() {
        intakeSubsystem.setIntakeSpeed(Constants.Speed.INTAKE/10);
        shooterSubsystem.setShooterSpeed(Constants.Speed.SHOOTER);
    }

    public boolean isFinished() {
        double pos = intakeSubsystem.intakeMotor.getPosition().getValue();
        if(pos - initalPosition >= 5){
            SmartDashboard.putBoolean("Intake Command", true);
        }
        return (pos - initalPosition >= 5);
    }
}