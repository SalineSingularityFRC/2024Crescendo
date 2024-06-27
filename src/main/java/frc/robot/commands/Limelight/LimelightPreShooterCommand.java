package frc.robot.commands.Limelight;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightPreShooterCommand extends SequentialCommandGroup{
    public LimelightPreShooterCommand(ShooterSubsystem shooter, IntakeSubsystem intake, ArmSubsystem arm, 
    Supplier<Double> position) {

    addCommands(
        intake.stopIntaking(), //maybe not needed
        arm.limelightShootTarget(position),
        new ReverseIntakeCommand(intake),
        intake.stopIntaking(),
        shooter.setShooterCoast(),
        shooter.autonStartUpShooter()
    );
   }
}
