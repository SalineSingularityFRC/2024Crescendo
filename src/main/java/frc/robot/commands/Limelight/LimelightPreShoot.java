package frc.robot.commands.Limelight;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public SwerveSubsystem swerve;
    public Limelight lime;
    public LimelightPreShoot(ShooterSubsystem shooter, SwerveSubsystem swerve, ArmSubsystem arm, 
        Limelight lime, IntakeSubsystem intake) {
        this.swerve = swerve;
        this.lime = lime;
        //double[] knownDistances = getKnownDistance(swerve, lime);
        //knownDistances[1] returns the index of the current distance value in the drive array
        double shootingPos = 0;//Constants.Limelight.knownShootingPositions[(int) knownDistances[1]];
        SmartDashboard.putNumber("shootingPosLimelight", shootingPos);
        addCommands(
            new LimelightPreShooterCommand(shooter, intake, arm, this::getKnownDistanceArmPos),
            new toSpeaker(swerve, lime)
        );
   }

    public double getKnownDistanceArmPos(){
        return this.swerve.findClosestDistance(this.lime.getDistanceToTagInFeet())[1];
    };
}
