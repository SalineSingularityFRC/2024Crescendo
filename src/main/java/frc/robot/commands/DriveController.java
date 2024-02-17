package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveController extends Command {
    private final SwerveSubsystem m_swerve;
    private final DoubleSupplier m_rotation, m_x, m_y;

    public DriveController(SwerveSubsystem swerve, DoubleSupplier rotation, DoubleSupplier x, DoubleSupplier y) {
        m_swerve = swerve;
        m_rotation = rotation;
        m_x = x;
        m_y = y;
        addRequirements(swerve);
    }

    public void execute() {
        m_swerve.drive(new SwerveSubsystem.SwerveRequest(m_rotation.getAsDouble(), m_x.getAsDouble(), m_y.getAsDouble()), true);
    }

    public boolean isFinished() {
        return false;
    }
}