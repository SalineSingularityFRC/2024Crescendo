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
    //private FieldOriented velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(1).withEnableFOC(true);
  
    private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    private final double manualSmallP = 1.3;
    private final double manualSmallI = 0;
    private final double manualSmallD = 0;
    private final double manualSmallS = 0.9; // counters static friction

    private final double motorShooter21P = 1.2;
    private final double motorShooter21I = 0;
    private final double motorShooter21D = 0;
    private final double motorShooter21S = 0.9; // counters static friction

    public ShooterSubsystem() {

        shooterMotor1 = new TalonFX(Constants.CanId.Arm.Motor.SHOOTER_1, Constants.Canbus.DEFAULT);
        shooterMotor2 = new TalonFX(Constants.CanId.Arm.Motor.SHOOTER_2, Constants.Canbus.DEFAULT);
        //shooterMotor2.setControl(new Follower(Constants.CanId.Arm.Motor.SHOOTER_1, false));

        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.kP = manualSmallP;
        slot1Configs.kI = manualSmallI;
        slot1Configs.kD = manualSmallD;
        slot1Configs.kS = manualSmallS;

        Slot1Configs slot1ConfigsShooter2 = new Slot1Configs();
        slot1ConfigsShooter2.kP = motorShooter21P;
        slot1ConfigsShooter2.kI = motorShooter21I;
        slot1ConfigsShooter2.kD = motorShooter21D;
        slot1ConfigsShooter2.kS = motorShooter21S;

        CurrentLimitsConfigs current = new CurrentLimitsConfigs();
        current.SupplyCurrentLimit = 20;
        current.SupplyCurrentLimitEnable = true;
        shooterMotor1.getConfigurator().apply(current);
        current.SupplyCurrentLimit = 40;
        shooterMotor2.getConfigurator().apply(current);
        
        shooterMotor1.getConfigurator().apply(slot1Configs);
        shooterMotor2.getConfigurator().apply(slot1ConfigsShooter2);

        setCoastMode();
    }

    public void setShooterSpeed(double speed) {
        SmartDashboard.putNumber("Ideal Speed", speed);
        SmartDashboard.putNumber("Shooter 1 Speed", getShooter1Speed());
        SmartDashboard.putNumber("Ideal Speed", getShooter2Speed());
        shooterMotor1.setControl(velocityVoltage.withVelocity(speed)
            .withFeedForward(0.05).withSlot(1));

        shooterMotor2.setControl(velocityVoltage.withVelocity(speed)
            .withFeedForward(0.05).withSlot(1));
        //shooterMotor1.set(speed/90.0);
    }
    

    
    public Command teleopShootCommand(){
        return run(
            () -> {
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
    public double getShooter1Speed() {
        return shooterMotor1.getVelocity().getValue();
    }
    public double getShooter2Speed() {
        return shooterMotor2.getVelocity().getValue();
    }

    public boolean shooterUpToSpeed(double multiplier) {
        return (getShooter1Speed() >= Constants.Speed.SHOOTER * multiplier) 
            && (getShooter2Speed() >= Constants.Speed.SHOOTER * multiplier) 
            && Math.abs(getShooter1Speed() - getShooter2Speed()) <= 10;
    }

    public Command stopShooting() {
        return runOnce(
                () -> {
                    shooterMotor1.stopMotor();
                    shooterMotor2.stopMotor();
                    currentSpeed = 0;
                });
    }

 
public Command autonStartUpShooter(){
    return new FunctionalCommand(
    () -> {
       
    }, 
    () -> {
        //currentSpeed = ;
        setShooterSpeed(Constants.Speed.SHOOTER);
    },
    (__unused) -> {
       
    },
    () -> {
        //0.8 Multiplier
      return shooterUpToSpeed(0.8);
    
    },
    this
    );
  }


}
