package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Main class to control the robot */
public class Gamepad {

  public String allianceColor = DriverStation.getAlliance().toString();

  private Timer highTargetTimer = new Timer();
  private Timer sliderTimer = new Timer();
  private Timer defaultTimer = new Timer();
  private Timer pickupFallenConeTimer = new Timer();
  private Timer clawCloseTimer = new Timer();
  private Timer clawOpenTimer = new Timer();

  private XboxController driveController;

  /**
   * @param driveControllerPort Controller port the drive controller is connected to, probably 0
   * @param armControllerPort Controller port the arm controller is connect to, probably 1
   */
  public Gamepad(int driveControllerPort) {
    driveController = new XboxController(driveControllerPort);
    //armController = new Joystick(armControllerPort);
  }

  // public void armPneumatics(
  //     LightSensor coneLightSensor,
  //     LightSensor cubeLightSensor,
  //     ArmSubsystem arm) {

  //   if (clawCloseTimer.get() >= 0.25) {
  //     if ((arm.smallArmMotorPosition + (Constants.Speed.ARM * 4.5))
  //         < Constants.Position.SmallArm.DEFAULT) {
  //       arm.smallArmMotorPosition += Constants.Speed.ARM * 4.5;
  //       arm.maintainPosition();
  //     }
  //     clawCloseTimer.stop();
  //     clawCloseTimer.reset();
  //   }
  // }

  public void swerveDrive(
      SwerveSubsystem robotSubsystem,
      Limelight limelight,
      ArmSubsystem arm,
      LightSensor cubeLightSensor,
      LightSensor coneLightSensor,
      SwerveOdometry odometry) {
    // limelight commands below

    // if (armController.getPOV() == 0) {
    //   limelight.turnToAngle(robotSubsystem);
    // }

    // if (driveController.getRawButtonPressed(Constants.Gamepad.Button.X)) {
    //   robotSubsystem.resetGyro();
    // }

    // if (armController.getRawButton(Constants.Gamepad.Button.LEFT)) {
    //   if (robotSubsystem.isCoast()) {
    //     robotSubsystem.setBrakeMode();
    //   } else {
    //     robotSubsystem.setCoastMode();
    //   }
    // }

    robotSubsystem.drive(
          new SwerveSubsystem.SwerveRequest(
            driveController.getRightTriggerAxis(),
            driveController.getLeftX(),
            driveController.getLeftY()),
            true);
  }

  public void arm(ShooterSubsystem newArm) {

    // if (highTargetTimer.get() >= 0.25) {
    //   arm.highTarget2();
    //   highTargetTimer.stop();
    //   highTargetTimer.reset();
    // }
    // if (sliderTimer.get() >= 0.7) {
    //   arm.sliderTarget2();
    //   sliderTimer.stop();
    //   sliderTimer.reset();
    // }
    // if (defaultTimer.get() >= 0.3) {
    //   arm.defaultTarget2();
    //   defaultTimer.stop();
    //   defaultTimer.reset();
    // }
    // if (pickupFallenConeTimer.get() >= 0.4) {
    //   arm.pickupFallenCone2();
    //   pickupFallenConeTimer.stop();
    //   pickupFallenConeTimer.reset();
    // }

    // if (driveController.getRawButtonPressed(Constants.Gamepad.Button.R_JOYSTICK)
    //     || armController.getRawButtonPressed(Constants.Gamepad.Button.X)) {
    //   arm.defaultTarget1();
    //   defaultTimer.start();
    // } else if (driveController.getRawButtonPressed(Constants.Gamepad.Button.L_JOYSTICK)
    //     || armController.getRawButtonPressed(Constants.Gamepad.Button.A)) {
    //   arm.pickupTarget();
    // } else if (driveController.getRawButtonPressed(Constants.Gamepad.Button.START)
    //     || armController.getRawButtonPressed(Constants.Gamepad.Button.Y)) {
    //   arm.highTarget1();
    //   highTargetTimer.start();
    // } else if (driveController.getRawButtonPressed(Constants.Gamepad.Button.Y)
    //     || armController.getRawButtonPressed(Constants.Gamepad.Button.START)) {
    //   arm.sliderTarget1();
    //   sliderTimer.start();
    // } else if (driveController.getRawButtonPressed(Constants.Gamepad.Button.BACK)
    //     || armController.getRawButtonPressed(Constants.Gamepad.Button.B)) {
    //   arm.mediumTarget();
    //   // } else if (armController.getRawButtonPressed(Constants.Back_Button)) {

    //   //   arm.pickupFallenCone1();
    //   //   pickupFallenConeTimer.start();
    // } else if (driveController.getRawAxis(Constants.Gamepad.Trigger.LEFT) > 0.05) {
    //   arm.setBigArmSpeed(-Constants.Speed.ARM);
    // } else if (driveController.getRawButton(Constants.Gamepad.Button.LEFT)) {
    //   arm.setBigArmSpeed(Constants.Speed.ARM);
    // } else
    
    // if (driveController.getRawAxis(Constants.Gamepad.Button.RIGHT) > 0.05) {
    //   newArm.setShooterSpeed(-Constants.Speed.ARM);
    //   //arm.setSmallArmSpeed(-Constants.Speed.ARM - .001);
    // } else if (driveController.getRawButton(Constants.Gamepad.Button.RIGHT)) {
    //   newArm.setShooterSpeed(Constants.Speed.ARM);
    // } else {
    //   newArm.setShooterSpeed(0);
    // }


      //arm.setSmallArmSpeed(Constants.Speed.ARM + .001);
    // } else {
    //   arm.maintainPosition();
    
  }
}
