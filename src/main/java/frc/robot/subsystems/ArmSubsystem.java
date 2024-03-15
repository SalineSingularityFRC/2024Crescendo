package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem.SwerveRequest;

public class ArmSubsystem extends SubsystemBase {
  public TalonFX armMotor1;
  private TalonFX armMotor2;

 // private MotionMagicVoltage positionTargetPreset = new MotionMagicVoltage(0).withSlot(1).withEnableFOC(true);

  private PositionVoltage positionTargetPreset = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
  //private PositionTorqueCurrentFOC positionTargetPreset = new PositionTorqueCurrentFOC(0).withSlot(0);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
  private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
  private MotionMagicConfigs motionMagicConfigsPresets;
  private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

  private final double manualBigP = .3;
  private final double manualBigI = 0;
  private final double manualBigD = 0;
  private final double manualBigS = 0.03;

  //OLD PID CONSTANTS
  private final double slot0P = 1.5;
  private final double slot0I = 0;
  private final double slot0D = 0.2;
  private final double slot0S = 0.0;



  public double armMotorPosition;

  public ArmSubsystem() {

    // REPLACE ARM IDS WITH THE REAL MOTOR IDS
    armMotor1 = new TalonFX(Constants.CanId.Arm.Motor.ARM_1, Constants.Canbus.DEFAULT);
    armMotor2 = new TalonFX(Constants.CanId.Arm.Motor.ARM_2, Constants.Canbus.DEFAULT);
    armMotor2.setControl(new Follower(Constants.CanId.Arm.Motor.ARM_1, true));

    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    limitSwitchConfigs.ReverseLimitAutosetPositionEnable = true;
    limitSwitchConfigs.ReverseLimitAutosetPositionValue = 0;
    limitSwitchConfigs.ForwardLimitAutosetPositionEnable = false;

    Slot0Configs slot0ConfigsBig = new Slot0Configs();
    slot0ConfigsBig.kP = slot0P;
    slot0ConfigsBig.kI = slot0I;
    slot0ConfigsBig.kD = slot0D;
    slot0ConfigsBig.kS = slot0S;

    Slot1Configs slot1ConfigsBig = new Slot1Configs();
    slot1ConfigsBig.kP = manualBigP;
    slot1ConfigsBig.kI = manualBigI;
    slot1ConfigsBig.kD = manualBigD;
    slot1ConfigsBig.kS = manualBigS;

    
    armMotor1.getConfigurator().apply(slot0ConfigsBig);
    armMotor1.getConfigurator().apply(slot1ConfigsBig);
    armMotor1.getConfigurator().apply(limitSwitchConfigs);
    motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
    motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40 / 30;
    motionMagicConfigsPresets.MotionMagicAcceleration = 100 / 40;
    motionMagicConfigsPresets.MotionMagicJerk = 900 / 30;

    armMotor1.getConfigurator().apply(motionMagicConfigsPresets);

    armMotorPosition = armMotor1.getPosition().getValue();

    setBrakeMode();
  }

  public void setArmSpeed(double speed) {
    //System.out.println("ARM SPEED");
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

  public void periodic(){

  }

  public Command stopArm() {
    return run(
        () -> {
          maintainArmPosition();
        });
  }

  public Command moveArmForward() {
    return run(
        () -> {
          setArmSpeed(Constants.Speed.ARM);
        });
  }

  public Command moveArmBackwards() {
    return run(
        () -> {
          setArmSpeed(-Constants.Speed.ARM);
        });
  }

  //RunOne or Run
  public Command ampTarget() {
    return runOnce(() -> {
      setPosition(Constants.Position.MainArm.AMP);
    });
  }


  public Command shootTarget(){
    return new FunctionalCommand(
    () -> {

    }, 
    () -> {
      setPosition(Constants.Position.MainArm.SHOOTING);
    },
    (_unused) -> {

    },
    ()->{
      return Math.abs(Constants.Position.MainArm.SHOOTING - armMotor1.getPosition().getValueAsDouble()) < 0.5;
    },
    this
    );
  }

  public Command autonShootTarget(double pos){
    return new FunctionalCommand(
    () -> {
      
    }, 
    () -> {
      setPosition(pos);
    },
    (_unused) -> {

    },
    ()->{
     
      return Math.abs(pos - armMotor1.getPosition().getValueAsDouble()) < 0.5;
    },
    this
    );
  }

 public Command climberTarget(){
    return new FunctionalCommand(
    () -> {}, 
    () -> {
      setPosition(Constants.Position.MainArm.CLIMBER);
    },
    (_unused) -> {},
    () -> {
      return Math.abs(Constants.Position.MainArm.CLIMBER - armMotor1.getPosition().getValueAsDouble()) < 0.5;
    },
    this
    );
  }

  public Command pickupTarget() {
    return runOnce(() -> {
      setPosition(Constants.Position.MainArm.PICKUP);
    });
  }

  public Command maintainArm() {
    return run(() -> {
      maintainArmPosition();
    });
  }

  public boolean isNotAtBottom(){
    return armMotor1.getReverseLimit().getValue() != ReverseLimitValue.ClosedToGround;
  }

  public boolean isAtBottom(){
    return armMotor1.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }


   public boolean isNotAtTop(){
    return armMotor1.getForwardLimit().getValue() != ForwardLimitValue.ClosedToGround;
  }

  public void maintainArmPosition() {
    //System.out.println("MAINTAIN");
   // armMotor1.getConfigurator().apply(motionMagicConfigsPresets);
    armMotor1.setControl(
        positionTargetPreset.withPosition(armMotorPosition).withFeedForward(0.03 * 12).withSlot(0));
  }
  
  public double getPosition(){
    return armMotor1.getPosition().getValueAsDouble();
  }
  
  public Command goHome(){
    return new FunctionalCommand(
    () -> {

    }, 
    () -> {
      setArmSpeed(-Constants.Speed.ARM*1.2);
    },
    (_unused) -> {

    },
    this::isAtBottom,
    this
    );
  }
}
