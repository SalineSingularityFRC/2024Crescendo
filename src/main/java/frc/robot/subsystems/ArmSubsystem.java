package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ArmSubsystem {
  public TalonFX smallArmMotor;
  public TalonFX bigArmMotor;
  public TalonFX bigArmMotor2;

  private MotionMagicVoltage positionTargetPreset =
      new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
  private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
  private TalonFXConfiguration talonFXConfigsManual = new TalonFXConfiguration();

  private MotionMagicConfigs motionMagicConfigsPresets;
  private MotionMagicConfigs motionMagicConfigsPresetsSmall;

  private MotionMagicConfigs motionMagicConfigsManual;

  private double bigArmPos;
  private double smallArmPos;

  // TODO: Move constants to constants file
  private final double presetSmallP = 2.0 * 20;
  private final double presetSmallI = 0.02 * 10;
  private final double presetSmallD = 0.02 * 10;
  private final double presetSmallS = 0.06 * 10;

  private final double presetBigP = 8.0 * 14;
  private final double presetBigI = 0.08 * 10;
  private final double presetBigD = 0.08 * 10;
  private final double presetBigS = 0.06 * 10;

  private final double manualSmallP = 0.18;
  private final double manualSmallI = 0;
  private final double manualSmallD = 0;
  private final double manualSmallS = 0.6; // counters static friction

  private final double manualBigP = .53;
  private final double manualBigI = 0;
  private final double manualBigD = 0;
  private final double manualBigS = 0.6;

  public double bigArmMotorPosition;
  public double smallArmMotorPosition;

  public ArmSubsystem(boolean bigArmIsInverted, boolean smallArmIsInverted) {
    smallArmMotor = new TalonFX(Constants.CanId.Arm.Motor.SMALL_ARM, Constants.Canbus.DEFAULT);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = Constants.CanId.CanCoder.SMALL_ARM;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    smallArmMotor.getConfigurator().apply(config);
    smallArmMotor.setInverted(smallArmIsInverted);

    bigArmMotor = new TalonFX(Constants.CanId.Arm.Motor.BIG_ARM, Constants.Canbus.DEFAULT);

    config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = Constants.CanId.CanCoder.BIG_ARM;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    bigArmMotor.getConfigurator().apply(config);
    bigArmMotor.setInverted(bigArmIsInverted);

    bigArmMotor2 = new TalonFX(Constants.CanId.Arm.Motor.BIG_ARM_2, Constants.Canbus.DEFAULT);
    bigArmMotor2.setControl(new Follower(Constants.CanId.Arm.Motor.BIG_ARM, true));

    Slot0Configs slot0ConfigsSmall = new Slot0Configs();
    Slot0Configs slot0ConfigsBig = new Slot0Configs();
    slot0ConfigsSmall.kP = presetSmallP;
    slot0ConfigsSmall.kI = presetSmallI;
    slot0ConfigsSmall.kD = presetSmallD;
    slot0ConfigsSmall.kS = presetSmallS;
    slot0ConfigsBig.kP = presetBigP;
    slot0ConfigsBig.kI = presetBigI;
    slot0ConfigsBig.kD = presetBigD;
    slot0ConfigsBig.kS = presetBigS;

    Slot1Configs slot1ConfigsSmall = new Slot1Configs();
    slot1ConfigsSmall.kP = manualSmallP;
    slot1ConfigsSmall.kI = manualSmallI;
    slot1ConfigsSmall.kD = manualSmallD;
    slot1ConfigsSmall.kS = manualSmallS;

    Slot1Configs slot1ConfigsBig = new Slot1Configs();
    slot1ConfigsBig.kP = manualBigP;
    slot1ConfigsBig.kI = manualBigI;
    slot1ConfigsBig.kD = manualBigD;
    slot1ConfigsBig.kS = manualBigS;

    bigArmMotor.getConfigurator().apply(slot0ConfigsBig);
    smallArmMotor.getConfigurator().apply(slot0ConfigsSmall);

    bigArmMotor.getConfigurator().apply(slot1ConfigsBig);
    smallArmMotor.getConfigurator().apply(slot1ConfigsSmall);

    motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsPresets.MotionMagicAcceleration = 100 / 40;
    motionMagicConfigsPresets.MotionMagicJerk = 900 / 30;

    motionMagicConfigsManual = talonFXConfigsManual.MotionMagic;
    motionMagicConfigsManual.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsManual.MotionMagicAcceleration = 70 / 40;
    motionMagicConfigsManual.MotionMagicJerk = 800 / 30;

    bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);

    motionMagicConfigsPresetsSmall = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresetsSmall.MotionMagicJerk = 900 / 30;
    motionMagicConfigsPresetsSmall.MotionMagicAcceleration = 100 / 50;
    motionMagicConfigsPresetsSmall.MotionMagicCruiseVelocity = .7;

    smallArmMotor.getConfigurator().apply(motionMagicConfigsPresetsSmall);

    bigArmMotorPosition = bigArmMotor.getPosition().getValue();
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();
  }

  public void setSmallArmSpeed(double speed) {

    smallArmMotor.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.05).withSlot(1));

    bigArmMotorPosition = bigArmMotor.getPosition().getValue();
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();
  }

  public void setBigArmSpeed(double speed) {

    bigArmMotor.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.05).withSlot(1));

    bigArmMotorPosition = bigArmMotor.getPosition().getValue();
    smallArmMotorPosition = smallArmMotor.getPosition().getValue();
  }

  public void setPosition(double smallArmAngle, double bigArmAngle) {
    smallArmPosition(smallArmAngle);
    bigArmPosition(bigArmAngle);
  }

  public void smallArmPosition(double smallArmAngle) {
    smallArmMotor.getConfigurator().apply(motionMagicConfigsPresetsSmall);
    smallArmMotor.setControl(
        positionTargetPreset.withPosition(smallArmAngle).withFeedForward(0.05).withSlot(0));

    smallArmMotorPosition = smallArmAngle;
  }

  public void bigArmPosition(double bigArmAngle) {
    bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
    bigArmMotor.setControl(
        positionTargetPreset.withPosition(bigArmAngle).withFeedForward(0.05).withSlot(0));

    bigArmMotorPosition = bigArmAngle;
  }

  public void highTarget(
      Timer timer) { // these will still be used for auton and limelight, which have the luxury of
    // calling this method over and over
    highTarget1();
    if (timer.get() >= 0.25) {
      highTarget2();
    }
  }

  public void
      highTarget1() { // these individual commands labeled 1 and 2 are for gamepad to call (so you
    // only need to press it once)
    bigArmPosition(Constants.Position.BigArm.HIGH);
  }

  public void highTarget2() {
    smallArmPosition(Constants.Position.SmallArm.HIGH);
  }

  public void sliderTarget(Timer timer) { // same comment as highTarget
    sliderTarget1();
    if (timer.get() >= 0.7) {
      sliderTarget2();
    }
  }

  public void sliderTarget1() { // same comment as highTarget1
    bigArmPosition(Constants.Position.BigArm.SLIDER);
  }

  public void sliderTarget2() {
    smallArmPosition(Constants.Position.SmallArm.SLIDER);
  }

  public void mediumTarget() {
    setPosition(Constants.Position.SmallArm.MEDIUM, Constants.Position.BigArm.MEDIUM);
  }

  public void pickupTarget() {
    setPosition(Constants.Position.SmallArm.PICKUP, Constants.Position.BigArm.PICKUP);
  }

  public void pickupFallenCone1() {
    bigArmPosition(Constants.Position.BigArm.PICKUPCONE);
  }

  public void pickupFallenCone2() {
    smallArmPosition(Constants.Position.SmallArm.PICKUP_CONE);
  }

  public void defaultTarget() {
    setPosition(Constants.Position.SmallArm.DEFAULT, Constants.Position.BigArm.DEFAULT);
  }

  public void defaultTargetTimer(Timer timer) {
    defaultTarget1();
    if (timer.get() >= 0.7) {
      defaultTarget2();
    }
  }

  public void defaultTarget1() {
    smallArmPosition(Constants.Position.SmallArm.DEFAULT);
  }

  public void defaultTarget2() {
    bigArmPosition(Constants.Position.BigArm.DEFAULT);
  }

  public void maintainPosition() {
    bigArmMotor.getConfigurator().apply(motionMagicConfigsPresets);
    smallArmMotor.getConfigurator().apply(motionMagicConfigsPresetsSmall);

    smallArmMotor.setControl(
        positionTargetPreset.withPosition(smallArmMotorPosition).withFeedForward(0.01).withSlot(0));
    bigArmMotor.setControl(
        positionTargetPreset.withPosition(bigArmMotorPosition).withFeedForward(0.01).withSlot(0));
  }
}
