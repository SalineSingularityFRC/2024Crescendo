package frc.robot.SwerveClasses;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.PID;

/**
 * This class owns a single Swerve Module's angle motor and is responsible for driving that motor to
 * a given angle
 */
public class SwerveAngle {
  /*
   * We need to have two class variables, a Falcon motor that we can use to control the angle of the module
   * and a variable for the zero poisition of the motor in radians (what value we need for it to be straight forward)
   */
  private double zeroPositionOffset;
  private TalonFX angleMotor;
  private PositionVoltage positionTarget;

  public int angleMotorID;
  /*
   * Our constructor needs to take a parameter that determines which CAN ID the falcon we are using has
   * and it needs to initialize the falcon motor and configure it (things like PID values and such)
   */

  public SwerveAngle(int angleMotorId, String canNetwork) {
    angleMotor = new TalonFX(angleMotorId, canNetwork);
    angleMotorID = angleMotorId;
    MotorOutputConfigs configs = new MotorOutputConfigs();
    
    // Factory reset before applying configs
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());

    zeroPositionOffset = 0;
    positionTarget = new PositionVoltage(0).withSlot(0).withFeedForward(0.2);
    
    PID turnPID = Constants.PidGains.SwerveModule.TURNING_PID_CONTROLLER;
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = turnPID.P;
    slot0Configs.kI = turnPID.I;
    slot0Configs.kD = turnPID.D;;
    slot0Configs.kS = turnPID.S;

    // CurrentLimitsConfigs current = new CurrentLimitsConfigs();
    // current.SupplyCurrentLimit = 15;
    // current.SupplyCurrentLimitEnable = true;

    configs.NeutralMode = NeutralModeValue.Brake;
    angleMotor.getConfigurator().apply(slot0Configs);
    angleMotor.getConfigurator().apply(configs);
    // angleMotor.getConfigurator().apply(current);
    
    angleMotor.setInverted(true);
  }

  /*
   * This enum provides the current state of the angle motor, we should either be matching the requested angle (Positive),
   * 180° off of it (Negative), or in the process of getting to one of those positions (Moving)
   */
  public enum AnglePosition {
    Positive,
    Negative,
    Moving
  }

  /*
   * This function takes an target angle (in radians) that we want the wheel to turn to
   * and updates the target position within the falcon motor so that it will move there.
   * It returns a value that indicates if we are in the target position, or 180° off of it,
   * or still working to get to one of those positions
   */

  // takes in an angle in and turns the wheel to that angle in the fastest way possible
  public AnglePosition setAngle(double targetAngle) {
    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Angle Clamped", getAngleClamped());
    double wheelPosition =
        getAngleClamped(); // the angle to set the wheel to minus the leftover full rotations
    double remainderRotations =
        getRemainderRotations(); // the additional rotations leftover from the wheel position
    double delta = wheelPosition - targetAngle;

    // This if else statement gets the closest angle to go to (absolute value)
    if (delta > Math.PI) {
      targetAngle += (2 * Math.PI);
    } else if (delta < -Math.PI) {
      targetAngle -= (2 * Math.PI);
    }

    // Recalculate the difference between the current angle according to the gyro and the target
    // angle according to the robot.
    delta = wheelPosition - targetAngle;
    AnglePosition currentPosition;

    /*
    These if else statements mean that if the target angle is closer  by turning in the other
    direction, do it. Then set the wheels to reverse because the wheel is now facing the other way

     tl;dr: turns the wheel the shortest possible distance by giving it the option to turn both
     counterclockwise and clockwise
    */
    // if (delta > (Math.PI / 2) || delta < -(Math.PI / 2)) {
    //   if (delta > (Math.PI / 2)) {
    //     targetAngle += Math.PI;
    //   } else if (delta < -(Math.PI / 2)) {
    //     targetAngle -= Math.PI;
    //   }
    //   currentPosition = AnglePosition.Negative;
    // } else {
      currentPosition = AnglePosition.Positive;
    // }

    targetAngle += remainderRotations;
    targetAngle += zeroPositionOffset;

    // Let's drive
    angleMotor.setControl(
        positionTarget.withPosition(
            Constants.MotorGearRatio.ANGLE * (targetAngle / (2 * Math.PI))));

    if(angleMotorID == 15){
      SmartDashboard.putNumber("Goofy Math", (Constants.MotorGearRatio.ANGLE * (targetAngle / (2 * Math.PI))) - angleMotor.getPosition().getValue());
    }
    if (Math.abs(delta % Math.PI) > Constants.AngleInaccuracy.MAX
        && Math.abs(delta % Math.PI) < Math.PI - Constants.AngleInaccuracy.MAX) {
      return AnglePosition.Moving; // Wheel is still in the process of turning
    }
    return currentPosition;
  }

  /*
   * Returns the angle (in radians) that the Talon is currently reporting we are in
   * minus our offset
   */
  public double getAngle() {
    double talonRadians = (angleMotor.getPosition().getValue() * 2 * Math.PI);
    double wheelRadians = talonRadians / Constants.MotorGearRatio.ANGLE;
    return wheelRadians - zeroPositionOffset;
  }

  /*
   * for getAngleClamped, return a value between [0, 2pi) that the talon says we are.
   * copied and pasted from the first lines of setAngle()
   */
  public double getAngleClamped() {

    if (getAngle() >= 0) {
      return getAngle() % (2 * Math.PI);
    } else {
      return (2 * Math.PI) - (Math.abs(getAngle()) % (2 * Math.PI));
    }
  }

  /*
   * Returns the number of rotations in either direction the Talon is currently spun
   */
  public double getRemainderRotations() {
    return getAngle() - getAngleClamped();
  }

  /*
   * Set the zero angle based on the current angle (in radians) that we are reading from an external source.
   */
  public void setZeroAngle(double currentAngle) {
    zeroPositionOffset += getAngle() - currentAngle;
  }
}
