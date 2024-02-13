package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/** Main class to control the robot */
public class Gamepad {
  public String allianceColor = DriverStation.getAlliance().toString();

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
}
