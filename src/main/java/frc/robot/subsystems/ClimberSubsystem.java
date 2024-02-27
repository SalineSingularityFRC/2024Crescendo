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

  private final double manualSmallP = 0.5;
  private final double manualSmallI = 0;
  private final double manualSmallD = 0;
  private final double manualSmallS = 0;

  public ClimberSubsystem() {
    climberMotor = new TalonFX(Constants.CanId.Arm.Motor.CLIMBER, Constants.Canbus.DEFAULT);

    Slot1Configs slot1ConfigsSmall = new Slot1Configs();
    slot1ConfigsSmall.kP = manualSmallP;
    slot1ConfigsSmall.kI = manualSmallI;
    slot1ConfigsSmall.kD = manualSmallD;
    slot1ConfigsSmall.kS = manualSmallS;
    climberMotor.getConfigurator().apply(slot1ConfigsSmall);

    setBrakeMode();
  }

  public void setClimberPosition(double climberPos) {
    climberMotor.setControl(positionTargetPreset.withPosition(climberPos).withFeedForward(0.1).withSlot(0));
  }

  public void setBrakeMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    climberMotor.getConfigurator().apply(motorOutputConfigs);
  }

  public Command moveClimberUp() {
    return new FunctionalCommand(
      () -> {},
      () -> {setClimberPosition(Constants.Position.Climber.UP);},
      (_unused) -> {},
      () -> {
        return Math.abs(Constants.Position.Climber.UP - climberMotor.getPosition().getValueAsDouble()) < 0.5;
      },
      this);
  }

  public Command moveClimberDown() {
    return new FunctionalCommand(
      () -> {},
      () -> {setClimberPosition(Constants.Position.Climber.DOWN);},
      (_unused) -> {},
      () -> {
        return Math.abs(Constants.Position.Climber.DOWN - climberMotor.getPosition().getValueAsDouble()) < 0.5;
      },
      this);
  }
}
