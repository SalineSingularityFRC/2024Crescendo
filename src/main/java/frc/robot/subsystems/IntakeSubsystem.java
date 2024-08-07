package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public TalonFX intakeMotor;
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  private final double manualSmallP = 0.36;
  private final double manualSmallI = 0;
  private final double manualSmallD = 0;
  private final double manualSmallS = 0.6; // counters static friction

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.CanId.Arm.Motor.INTAKE, Constants.Canbus.DEFAULT);

    Slot1Configs slot1ConfigsSmall = new Slot1Configs();
    slot1ConfigsSmall.kP = manualSmallP;
    slot1ConfigsSmall.kI = manualSmallI;
    slot1ConfigsSmall.kD = manualSmallD;
    slot1ConfigsSmall.kS = manualSmallS;
    intakeMotor.getConfigurator().apply(slot1ConfigsSmall);

    setBrakeMode();
  }

  public void setIntakeSpeed(double speed) {
  
    intakeMotor.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.05).withSlot(1));
  }

  //rotations per second
  public double getIntakeSpeed() {
    return intakeMotor.getVelocity().getValue();
  }


  public void setBrakeMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intakeMotor.getConfigurator().apply(motorOutputConfigs);
  }


  public void setCoastMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    intakeMotor.getConfigurator().apply(motorOutputConfigs);
  }

  public Command setBrake() {
    return runOnce(
        () -> {
          setBrakeMode();
          
        });
  }
  public Command setCoast() {
    return runOnce(
        () -> {
          setCoastMode();
          
        });
  }
  public Command stopIntaking() {
    return runOnce(
        () -> {
          intakeMotor.stopMotor();
          
        });
  }



  public Command autonStopIntaking(ShooterSubsystem shooter){
    return new FunctionalCommand(
    () -> {

    }, 
    () -> {
      intakeMotor.stopMotor();
    },
    (_unused) -> {
      
    },
    () -> {
      return getIntakeSpeed() <= 5;
    },
    this
    );
  }
  

  public Command reverseIntake() {
    return run(
        () -> {
          setIntakeSpeed(-Constants.Speed.INTAKE);
        });
  }
  public Command startIntake() {
    return run(
        () -> {
          setIntakeSpeed(Constants.Speed.INTAKE);
        });
  }

public Command intakeContinue() {
    return run(
        () -> {
       
      });
  }
}
