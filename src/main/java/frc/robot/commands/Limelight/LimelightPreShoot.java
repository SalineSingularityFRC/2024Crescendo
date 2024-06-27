package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.commands.Auton.PreShooter;
import frc.robot.commands.Auton.Shooter;
import frc.robot.commands.Teleop.toSpeaker;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightPreShoot extends SequentialCommandGroup {
    public LimelightPreShoot(ShooterSubsystem shooter, SwerveSubsystem swerve, ArmSubsystem arm,
            Limelight lime, IntakeSubsystem intake) {
        double[] knownDistances = getKnownDistance(swerve, lime);
        // knownDistances[1] returns the index of the current distance value in the
        // drive array
        double shootingPos = Constants.Limelight.knownShootingPositions[(int) knownDistances[1]];
        addCommands(
                new PreShooter(shooter, intake, arm, shootingPos),
                new toSpeaker(swerve, lime));
    }

    public double[] getKnownDistance(SwerveSubsystem swerve, Limelight lime) {
        return swerve.findClosestDistance(lime.getDistanceToTagInFeet());
    }
}
