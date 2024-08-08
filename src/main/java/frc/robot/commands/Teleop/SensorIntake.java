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
    private LaserCan laserCanSensor1;
    private LaserCan laserCanSensor2;
    private double measurement1;
    private double measurement2;
    private double initalPosition;

    public SensorIntake(IntakeSubsystem intake, LaserCan lcs1, LaserCan lcs2) {
        intakeSubsystem = intake;
        laserCanSensor1 = lcs1;
        laserCanSensor2 = lcs2;
        addRequirements(intakeSubsystem);
    }

    public void initialize(){
        this.initalPosition = intakeSubsystem.intakeMotor.getPosition().getValue();
    }

    public void execute() {
        measurement1 = laserCanSensor1.getMeasurement().distance_mm;
        measurement2 = laserCanSensor2.getMeasurement().distance_mm;
        
        if (measurement1 <= Constants.LaserCan.INTAKE_WIDTH_MM - Constants.LaserCan.INTAKE_TOLERANCE_MM_1
        && measurement1 != -1) {
            intakeSubsystem.setIntakeSpeed(Constants.Speed.INTAKE/5);
        }
        else {
            intakeSubsystem.setIntakeSpeed(Constants.Speed.INTAKE);
        }
    }

    public boolean isFinished() {
        return (measurement2 <= Constants.LaserCan.INTAKE_WIDTH_MM - Constants.LaserCan.INTAKE_TOLERANCE_MM_2
         && measurement2 != -1);
    }
}