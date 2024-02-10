package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public TalonFX armMotor1;
    public TalonFX armMotor2;

    private MotionMagicVoltage positionTargetPreset = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private TalonFXConfiguration talonFXConfigsPreset = new TalonFXConfiguration();
    private TalonFXConfiguration talonFXConfigsManual = new TalonFXConfiguration();

    private MotionMagicConfigs motionMagicConfigsPresets;
    private MotionMagicConfigs motionMagicConfigsPresetsSmall;

    private MotionMagicConfigs motionMagicConfigsManual;

    private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    private final double manualSmallP = 0.36;
    private final double manualSmallI = 0;
    private final double manualSmallD = 0;
    private final double manualSmallS = 0.6; // counters static friction

    public ShooterSubsystem() {

        armMotor1 = new TalonFX(Constants.CanId.Arm.Motor.SHOOTER_1, Constants.Canbus.DEFAULT);
        armMotor2 = new TalonFX(Constants.CanId.Arm.Motor.SHOOTER_2, Constants.Canbus.DEFAULT);

        // TRUE FOR THE BOTTOM MOTORS
        // FALSE FOR SHOOTER
        armMotor2.setControl(new Follower(Constants.CanId.Arm.Motor.SHOOTER_1, false));

        Slot1Configs slot1ConfigsSmall = new Slot1Configs();
        slot1ConfigsSmall.kP = manualSmallP;
        slot1ConfigsSmall.kI = manualSmallI;
        slot1ConfigsSmall.kD = manualSmallD;
        slot1ConfigsSmall.kS = manualSmallS;

        armMotor1.getConfigurator().apply(slot1ConfigsSmall);

        motionMagicConfigsPresets = talonFXConfigsPreset.MotionMagic;
        motionMagicConfigsPresets.MotionMagicCruiseVelocity = 40 / 30;
        motionMagicConfigsPresets.MotionMagicAcceleration = 100 / 40;
        motionMagicConfigsPresets.MotionMagicJerk = 900 / 30;

        motionMagicConfigsManual = talonFXConfigsManual.MotionMagic;
        motionMagicConfigsManual.MotionMagicCruiseVelocity = 40 / 30;
        motionMagicConfigsManual.MotionMagicAcceleration = 70 / 40;
        motionMagicConfigsManual.MotionMagicJerk = 800 / 30;

        armMotor1.getConfigurator().apply(motionMagicConfigsPresets);

        setBrakeMode();
    }

    public void setShooterSpeed(double speed) {
        armMotor1.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.05).withSlot(1));
    }

    public void setBrakeMode() {
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        armMotor1.getConfigurator().apply(motorOutputConfigs);
        armMotor2.getConfigurator().apply(motorOutputConfigs);
    }

    public Command stopMotor() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(
                () -> {
                    setShooterSpeed(0);
                });
    }

    public Command moveMotor() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(
                () -> {
                    setShooterSpeed(Constants.Speed.SHOOTER);
                });
    }

}
