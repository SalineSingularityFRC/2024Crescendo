package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  public TalonFX climberMotor;
  private PositionVoltage positionTargetPreset = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  private final double manualSmallP = 0.25;
  private final double manualSmallI = .0025;
  private final double manualSmallD = 0;
  private final double manualSmallS = 0;

  public double climberMotorPosition;
  public ClimberSubsystem() {
    climberMotor = new TalonFX(Constants.CanId.Arm.Motor.CLIMBER, Constants.Canbus.DEFAULT);

    Slot1Configs slot1ConfigsSmall = new Slot1Configs();
    slot1ConfigsSmall.kP = manualSmallP;
    slot1ConfigsSmall.kI = manualSmallI;
    slot1ConfigsSmall.kD = manualSmallD;
    slot1ConfigsSmall.kS = manualSmallS;
    climberMotor.getConfigurator().apply(slot1ConfigsSmall);

    climberMotorPosition = climberMotor.getPosition().getValue();
    setBrakeMode();
  }

  public void setClimberSpeed(double speed) {
    climberMotor.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.1).withSlot(1));
    climberMotorPosition = climberMotor.getPosition().getValue();
  }

  public void setBrakeMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    climberMotor.getConfigurator().apply(motorOutputConfigs);
  }

  public Command moveClimberUp() {
    return run(() -> {
      setClimberSpeed(Constants.Speed.CLIMBER);
    });
  }

  public Command moveClimberDown() {
    return run(() -> {
      setClimberSpeed(-Constants.Speed.CLIMBER);
    });
  }

  public void maintainClimberPosition() {
   
    climberMotor.setControl(
        positionTargetPreset.withPosition(climberMotorPosition).withFeedForward(0.03 * 12).withSlot(1));
  }
  public Command maintainClimberPosCommand() {
    return run(() -> {
      maintainClimberPosition();
    });
  }
  
}
