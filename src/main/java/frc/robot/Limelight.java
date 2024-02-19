package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId.Swerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveRequest;

public class Limelight extends SubsystemBase{
  private NetworkTable table;
  public NetworkTableEntry tx, ty, ta, tv, ledMode, camMode, pipeLine, crop;

  private double[] botpose, targetspace;
  public double poseX, poseY, poseYaw;
  public double targetposeX, targetposeZ, targetposeYaw;
  public double[] tid;
  public double tl, cl, limeLatency;

  public boolean isTurningDone;
  public final double minimumSpeed = 0.06;
  PIDController driveController;
  public PIDController turnController;
  PIDController scoreDriveController;

  public Timer scoringTimer = new Timer();
  public Timer pickupTimer = new Timer();

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
    ty = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
    ta = table.getEntry("ta"); // target area (0-100%)
    tv = table.getEntry("tv"); // 0 = no target found or 1 = target found

    // swap the limelight between vision processing (0) and drive camera (1)
    camMode = table.getEntry("camMode");

    // state of LEDs: 1.0 - on, 2.0 - blink, 3.0 - off
    ledMode = table.getEntry("ledMode");

    pipeLine = table.getEntry("pipeline");

    botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
    // xyz are in meters
    poseX = botpose[0]; // Points up the long side of the field
    poseY = botpose[1]; // Points toward short side of the field
    poseYaw = botpose[5] * (Math.PI/180); // angle of the robot 0 is straight
    tid = table.getEntry("tid").getDoubleArray(new double[6]); // id of the primary in view April tag

    // the robots position based on the primary in view april tag, (0, 0, 0) at center of the april tag
    targetspace = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    targetposeX = targetspace[0]; // to the right of the target from front face
    targetposeZ = targetspace[2]; // pointing out of the april tag
    targetposeYaw = targetspace[5]; 

    tl = table.getEntry("tl").getDouble(0); // targeting latency
    cl = table.getEntry("cl").getDouble(0); // capture latency
    limeLatency = tl + cl; // total latency for the pipeline (ms)

    double[] drive_gains = Constants.PidGains.Limelight.DRIVE_CONTROLLER;
    driveController =
        new PIDController(drive_gains[0], drive_gains[1], drive_gains[2]); // 0.0056 orginally
    driveController.setSetpoint(-18);

    // driveController.setTolerance(0.5);
    double[] turn_gains = Constants.PidGains.Limelight.TURN_CONTROLLER;
    turnController = new PIDController(turn_gains[0], turn_gains[1], turn_gains[2]);
    turnController.setSetpoint(0);

    double[] score_drive_gains = Constants.PidGains.Limelight.SCORE_DRIVE_CONTROLLER;
    scoreDriveController =
        new PIDController(score_drive_gains[0], score_drive_gains[1], score_drive_gains[2]);
    scoreDriveController.setSetpoint(1.7); // FIND RIGHT TA VALUE

    // driveController.setTolerance(0.5);
    setCamMode(0); // set to vision mode
    ledOff();
    setpipeline(0);
  }

  // turns on the LEDs
  public void ledOn() {
    ledMode.setDouble(3.0);
  }

  // turns off the LEDs
  public void ledOff() {
    ledMode.setDouble(0.0);
  }

  // method to switch camera between drive mode and vision mode
  public void setCamMode(double mode) {
    camMode.setDouble(mode);
  }

  // method to change between pipeLines, takes an int and a LimeLight object
  public void setpipeline(int pipe) {
    pipeLine.setNumber(pipe);
  }

  public boolean getIsTargetFound() {
    double a = ta.getDouble(0);
    if (a <= 0.05) {
      return false;
    } else {
      return true;
    }
  }

  // public boolean pickup(
  //     SwerveSubsystem drive,
  //     ArmSubsystem arm,
  //     LightSensor cubeLightSensor,
  //     LightSensor coneLightSensor,
  //     boolean isCube,
  //     boolean auton) {

  //   if (isCube) {
  //     setpipeline(2);
  //   } else {
  //     setpipeline(1); // is cone
  //   }

  //   arm.pickupTarget();
  //   ledOff();


  //   if (tx.getDouble(0) < 6.5 || tx.getDouble(0) > 11.5) {
  //     isTurningDone = false;
  //   }
  //   if (!isTurningDone) {
  //     isTurningDone = turnAngle(drive, true);
  //   } else {

  //     double y = driveController.calculate(ty.getDouble(0));
  //     if (y > 0) y += 0.06;
  //     if (y < 0) y -= 0.06;

  //     if (this.pickupTimer.get() == 0) {
  //       double speed = turnController.calculate(tx.getDouble(0));
  //       drive.drive(new SwerveRequest(-speed, 0, -y * 2.5), false);
  //     }
  //   }

  //   return false;
  // }

  // public boolean turnAngle(SwerveSubsystem drive, boolean pickup) {
  //   if (getIsTargetFound()) {

  //     double speed = turnController.calculate(tx.getDouble(0));
  //     if (speed > 0) speed += 0.05;
  //     if (speed < 0) speed -= 0.05;

  //     if (turnController.atSetpoint()) {
  //       turnController.reset();
  //       return true; // we are angled correctly
  //     }
  //     if (pickup) {
  //       if (pickupTimer.get() == 0) drive.drive(new SwerveRequest(-speed, 0, 0), false);
  //     } else {
  //       drive.drive(new SwerveRequest(-speed, 0, 0), false);
  //     }
  //   }

  //   return false;
  // }

  public boolean score(
      SwerveSubsystem drive, ArmSubsystem arm, boolean isCube) {
    ledOn();
    if (isCube) {
      setpipeline(0);
    } else {
      setpipeline(3);
    }
    if (tx.getDouble(0) < 6.5 || tx.getDouble(0) > 11.5) {
      isTurningDone = false;
    }

    return false;
  }

  // public void turnToAngle(SwerveSubsystem drive) {
  //   double robotAngle = (drive.getRobotAngle() % (Math.PI * 2)) * (180 / Math.PI); // in degree
  //   double rotation = (180 - robotAngle) * 0.0061;
  //   if (rotation >= minimumSpeed) {
  //     rotation -= minimumSpeed;
  //   } else if (rotation <= -minimumSpeed) {
  //     rotation += minimumSpeed;
  //   } else {
  //     rotation = 0;
  //     return;
  //   }

  //   drive.drive(new SwerveRequest(rotation, 0, 0), false);
  // }

  public Command scoreRight(SwerveSubsystem d) {
    return run(
    () -> {
      turnRobot(d);
    });
  }

  public Command turnRobot(SwerveSubsystem d){
    return new FunctionalCommand(
    () -> {

    }, 
    () -> {
      setpipeline(0);
      //YAW
      double pos = table.getEntry("botpose_targetspace").getDoubleArray(new double[6])[5];
      System.out.println(pos);
      double rotation = turnController.calculate(pos);
      d.drive(new SwerveRequest(rotation, 0, 0), false);
    },
    (_unused) -> {

    },
    turnController::atSetpoint,
    this, d
    );
  }

  public boolean tagAlign() {
    if((targetposeX >= -0.075 && targetposeX <= 0.075)) {
      return true;
    }
    return false;
  }

}
