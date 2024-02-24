package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public TalonFX shooterMotor1;
    private TalonFX shooterMotor2;

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
    private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    private final double manualSmallP = 1.3;
    private final double manualSmallI = 0;
    private final double manualSmallD = 0;
    private final double manualSmallS = 0.9; // counters static friction

    public ShooterSubsystem() {

        shooterMotor1 = new TalonFX(Constants.CanId.Arm.Motor.SHOOTER_1, Constants.Canbus.DEFAULT);
        shooterMotor2 = new TalonFX(Constants.CanId.Arm.Motor.SHOOTER_2, Constants.Canbus.DEFAULT);
        shooterMotor2.setControl(new Follower(Constants.CanId.Arm.Motor.SHOOTER_1, false));

        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.kP = manualSmallP;
        slot1Configs.kI = manualSmallI;
        slot1Configs.kD = manualSmallD;
        slot1Configs.kS = manualSmallS;

        CurrentLimitsConfigs current = new CurrentLimitsConfigs();
        current.SupplyCurrentLimit = 30;
        current.SupplyCurrentLimitEnable = true;
        shooterMotor1.getConfigurator().apply(current);
        shooterMotor2.getConfigurator().apply(current);
        
        shooterMotor1.getConfigurator().apply(slot1Configs);

        setCoastMode();
    }

    public void setShooterSpeed(double speed) {
        shooterMotor1.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.05).withSlot(1));
    }

    public void setCoastMode() {
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        shooterMotor1.getConfigurator().apply(motorOutputConfigs);
        shooterMotor2.getConfigurator().apply(motorOutputConfigs);
    }

    public double getShooterSpeed() {
        return shooterMotor1.getVelocity().getValue();
    }

    public Command stopShooting() {
        return runOnce(
                () -> {
                    shooterMotor1.stopMotor();
                });
    }

    public Command startShooting() {
        return run(
                () -> {
                    setShooterSpeed(Constants.Speed.SHOOTER);
                });
    }

}
