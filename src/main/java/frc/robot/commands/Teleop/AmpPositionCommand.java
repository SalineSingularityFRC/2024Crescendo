package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpPositionCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private ArmSubsystem arm;
    public AmpPositionCommand(ShooterSubsystem shooter, ArmSubsystem arm) {
        shooterSubsystem = shooter;
        this.arm = arm;
        addRequirements(shooterSubsystem);
    }

    public void execute() {
        arm.setPosition(Constants.Position.MainArm.AMP);
    }

    public boolean isFinished() {
        return Math.abs(Constants.Position.MainArm.AMP - arm.armMotor1.getPosition().getValueAsDouble()) < 0.5;
    }
}