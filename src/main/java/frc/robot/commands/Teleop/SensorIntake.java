package frc.robot.commands.Teleop;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

// For moving the intake back during shooting
public class SensorIntake extends Command {
    private IntakeSubsystem intakeSubsystem;
    private LaserCan laserCanSensor;
    private double measurement;
    private double initalPosition;

    public SensorIntake(IntakeSubsystem intake, LaserCan lcs) {
        intakeSubsystem = intake;
        laserCanSensor = lcs;
        addRequirements(intakeSubsystem);
    }

    public void initialize(){
        this.initalPosition = intakeSubsystem.intakeMotor.getPosition().getValue();
        
    }

    public void execute() {
        measurement = laserCanSensor.getMeasurement().distance_mm;
        intakeSubsystem.setIntakeSpeed(Constants.Speed.INTAKE);
    }

    public boolean isFinished() {
        return (measurement <= 100
         && measurement != -1);
    }
}