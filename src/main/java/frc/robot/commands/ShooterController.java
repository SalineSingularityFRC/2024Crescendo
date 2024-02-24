package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterController extends Command {
    private ShooterSubsystem shooterSubsystem;

    public ShooterController(ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
        addRequirements(shooterSubsystem);
    }

    public void execute() {
        shooterSubsystem.setShooterSpeed(Constants.Speed.SHOOTER);
    }

    public boolean isFinished() {
        return (shooterSubsystem.getShooterSpeed() >= Constants.Speed.SHOOTER * 0.9); 
    }
}