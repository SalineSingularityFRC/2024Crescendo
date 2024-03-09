package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveController extends Command {
    private final SwerveSubsystem m_swerve;
    private final DoubleSupplier m_rotation, m_x, m_y;
    private double multiplier;
    public DriveController(SwerveSubsystem swerve, DoubleSupplier rotation, DoubleSupplier x, DoubleSupplier y, double multiplier) {
        this.multiplier = multiplier;
        m_swerve = swerve;
        m_rotation = rotation;
        m_x = x;
        m_y = y;
        addRequirements(swerve);
    }

    public double fixDecimalTo2Places(double number){
        return Math.round(number * 100.0) / 100.0;
    }
    public void execute() {
        m_swerve.drive(new SwerveSubsystem.SwerveRequest(fixDecimalTo2Places(-m_rotation.getAsDouble() * multiplier),fixDecimalTo2Places(-m_x.getAsDouble() * multiplier),fixDecimalTo2Places(-m_y.getAsDouble()) * multiplier), true);
    }
  

    public boolean isFinished() {
        return false;
    }
}