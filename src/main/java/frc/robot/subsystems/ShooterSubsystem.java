package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    public TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    public double currentSpeed;
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
        current.SupplyCurrentLimit = 50;
        current.SupplyCurrentLimitEnable = true;
        shooterMotor1.getConfigurator().apply(current);
        shooterMotor2.getConfigurator().apply(current);
        
        shooterMotor1.getConfigurator().apply(slot1Configs);
        
        setCoastMode();
    }

    public void setShooterSpeed(double speed) {
        //shooterMotor1.setControl(velocityVoltage.withVelocity(speed).withFeedForward(0.05).withSlot(1));
        shooterMotor1.set(speed/90.0);
    }
    WaitCommand wait = new WaitCommand(0.5);
    public Command teleopShootCommand(){
        return run(
            () -> {
                wait.initialize();
                setShooterSpeed(Constants.Speed.SHOOTER);
            }
        );
    }

    public void setCoastMode() {
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        shooterMotor1.getConfigurator().apply(motorOutputConfigs);
        shooterMotor2.getConfigurator().apply(motorOutputConfigs);
    }

    public void setBrakeMode() {
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        shooterMotor1.getConfigurator().apply(motorOutputConfigs);
        shooterMotor2.getConfigurator().apply(motorOutputConfigs);
    }

    // public void periodic(){
    //     if(currentSpeed == 0){
    //         shooterMotor1.stopMotor();
    //     } else {
    //         setShooterSpeed(currentSpeed);
    //     }
       
    // }
    public Command setShooterBrake(){
        return runOnce(
            () -> {
                setBrakeMode();
            }
        );
    }
    public Command setShooterCoast(){
        return runOnce(
            () -> {
                setCoastMode();
            }
        );
    }
    public double getShooterSpeed() {
        return shooterMotor1.getVelocity().getValue();
    }

    public Command stopShooting() {
        return runOnce(
                () -> {
                    shooterMotor1.stopMotor();
                    currentSpeed = 0;
                });
    }

    public Command startShooting() {
        return runOnce(
                () -> {
                    currentSpeed = (Constants.Speed.SHOOTER);
                });
    }

public Command autonStartUpShooter(){
    return new FunctionalCommand(
    () -> {
        SmartDashboard.putBoolean("Auton Start UP Shooter", false);
    }, 
    () -> {
        //currentSpeed = ;
        setShooterSpeed(Constants.Speed.SHOOTER);
    },
    (__unused) -> {
        SmartDashboard.putBoolean("Auton Start UP Shooter", true);
    },
    () -> {
      return getShooterSpeed() >= Constants.Speed.SHOOTER * 0.4;
    
    },
    this
    );
  }


}
