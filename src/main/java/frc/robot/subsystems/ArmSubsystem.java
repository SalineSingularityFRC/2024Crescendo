package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX armMotor1;
  private TalonFX armMotor2;

  private MotionMagicVoltage positionTargetPreset = new MotionMagicVoltage(0).withSlot(1).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
  private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
  private MotionMagicConfigs motionMagicConfigsPresets;
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  private final double manualBigP = .25;
  private final double manualBigI = 0;
  private final double manualBigD = 0;
  private final double manualBigS = 0.6;

  public double armMotorPosition;

  public ArmSubsystem() {

    // REPLACE ARM IDS WITH THE REAL MOTOR IDS
    armMotor1 = new TalonFX(Constants.CanId.Arm.Motor.ARM_1, Constants.Canbus.DEFAULT);
    armMotor2 = new TalonFX(Constants.CanId.Arm.Motor.ARM_2, Constants.Canbus.DEFAULT);
    armMotor2.setControl(new Follower(Constants.CanId.Arm.Motor.ARM_1, true));

    Slot1Configs slot1ConfigsBig = new Slot1Configs();
    slot1ConfigsBig.kP = manualBigP;
    slot1ConfigsBig.kI = manualBigI;
    slot1ConfigsBig.kD = manualBigD;
    slot1ConfigsBig.kS = manualBigS;

    armMotor1.getConfigurator().apply(slot1ConfigsBig);

    motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsPresets.MotionMagicAcceleration = 100 / 40;
    motionMagicConfigsPresets.MotionMagicJerk = 900 / 30;

    armMotor1.getConfigurator().apply(motionMagicConfigsPresets);

    armMotorPosition = armMotor1.getPosition().getValue();

    setBrakeMode();
  }

  public void setArmSpeed(double speed) {
    armMotor1.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.1).withSlot(1));
    armMotorPosition = armMotor1.getPosition().getValue();
  }

  public void setPosition(double bigArmAngle) {
    armMotor1.setControl(
        positionTargetPreset.withPosition(bigArmAngle).withFeedForward(0.1).withSlot(0));

    armMotorPosition = bigArmAngle;
  }

  public void setBrakeMode() {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    armMotor1.getConfigurator().apply(motorOutputConfigs);
    armMotor2.getConfigurator().apply(motorOutputConfigs);
  }

  public Command stopMotor() {
    return run(
        () -> {
          maintainPosition();
        });
  }

  public Command moveMotorForward() {
    return run(
        () -> {
          setArmSpeed(Constants.Speed.ARM);
        });
  }

  public Command moveMotorBackward() {
    return run(
        () -> {
          setArmSpeed(-Constants.Speed.ARM);
        });
  }

  public void maintainPosition() {
    armMotor1.setControl(
        positionTargetPreset.withPosition(armMotorPosition).withFeedForward(0.1).withSlot(0));
  }
}
