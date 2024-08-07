package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShootCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    public StartShootCommand(ShooterSubsystem shooter) {
        shooterSubsystem = shooter;
        addRequirements(shooterSubsystem);
    }

    public void execute() {
        shooterSubsystem.setShooterSpeed(Constants.Speed.SHOOTER);
     
    }

    public boolean isFinished() {
        //Multplier 0.9
        return (shooterSubsystem.shooterUpToSpeed(0.9)); 
    }
}