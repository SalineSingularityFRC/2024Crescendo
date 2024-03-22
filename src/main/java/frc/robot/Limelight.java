package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId.Swerve;
import frc.robot.commands.Auton.Shooter;
import frc.robot.commands.Teleop.ShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveRequest;

public class Limelight extends SubsystemBase{
  private NetworkTable table;
  public NetworkTableEntry TX, TY, TA, TV, TID, ledMode, camMode, pipeLine, crop;

  public NetworkTableEntry botpose, targetspace;
  public double poseX, poseY, poseYaw;
  public double targetPoseX, targetPoseZ, targetPoseYaw, targetPoseY;
  public double tid;

  public double limeLatency;
  public double tx, ty, ta, tv;

  public boolean isTurningDone;
  public final double minimumSpeed = 0.06;
  PIDController driveController;
  public PIDController turnController;
  PIDController scoreDriveController;

  public Timer scoringTimer = new Timer();
  public Timer pickupTimer = new Timer();

  public Limelight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    TX = table.getEntry("tx"); // Horizontal Offset From Crosshair To Target (-29.8 to 29.8 degrees)
    TY = table.getEntry("ty"); // Vertical Offset From Crosshair To Target (-24.85 to 24.85 degrees)
    TA = table.getEntry("ta"); // target area (0-100%)
    TV = table.getEntry("tv"); // 0 = no target found or 1 = target found
    TID = table.getEntry("tid"); // id of the primary in view April tag

    ta = TX.getDouble(0.0);
    ty = TY.getDouble(0.0);
    ta = TA.getDouble(0.0);
    tv = TV.getDouble(0.0);
    tid = TID.getDouble(0);

    // swap the limelight between vision processing (0) and drive camera (1)
    camMode = table.getEntry("camMode");

    // state of LEDs: 1.0 - on, 2.0 - blink, 3.0 - off
    ledMode = table.getEntry("ledMode");

    pipeLine = table.getEntry("pipeline");

    // Location of the robot on the field with the orgin at the blue driver station
    botpose = table.getEntry("botpose_wpiblue");
    // xyz are in meters
    poseX = botpose.getDoubleArray(new double[6])[0]; // Points up the long side of the field
    poseY = botpose.getDoubleArray(new double[6])[1]; // Points toward short side of the field
    poseYaw = botpose.getDoubleArray(new double[6])[5] * (Math.PI/180); 

    // the robots position based on the primary in view april tag, (0, 0, 0) at center of the april tag
    targetspace = table.getEntry("targetpose_cameraspace");
    targetPoseX = targetspace.getDoubleArray(new double [6])[0]; // to the right of the target from front face
    targetPoseY = targetspace.getDoubleArray(new double [6])[1]; 
    targetPoseZ = targetspace.getDoubleArray(new double [6])[2]; // pointing out of the april tag
    targetPoseYaw = targetspace.getDoubleArray(new double[6])[5] * (Math.PI/180); 

    //limeLatency = botpose.getDoubleArray(new double[6])[6];

    PID drive_gains = Constants.PidGains.Limelight.DRIVE_CONTROLLER;
    driveController =
        new PIDController(drive_gains.P, drive_gains.I, drive_gains.D); // 0.0056 orginally
    driveController.setSetpoint(-18);

    // driveController.setTolerance(0.5);
    PID turn_gains = Constants.PidGains.Limelight.TURN_CONTROLLER;
    turnController = new PIDController(turn_gains.P, turn_gains.I, turn_gains.D);
    turnController.setSetpoint(0);

    PID score_drive_gains = Constants.PidGains.Limelight.SCORE_DRIVE_CONTROLLER;
    scoreDriveController =
        new PIDController(score_drive_gains.P, score_drive_gains.I, score_drive_gains.D);
    scoreDriveController.setSetpoint(1.7); // FIND RIGHT TA VALUE

    // driveController.setTolerance(0.5);
    setCamMode(0); // set to vision mode
    ledOff();
    setpipeline(0);
  }

  // turns on the LEDs
  public void ledOn() {
    ledMode.setNumber(3);
  }

  // turns off the LEDs
  public void ledOff() {
    ledMode.setNumber(0);
  }

  // method to switch camera between drive mode and vision mode
  public void setCamMode(double mode) {
    camMode.setNumber(mode);
  }

  // method to change between pipeLines, takes an int and o LimeLight object
  public void setpipeline(int pipe) {
    pipeLine.setNumber(pipe);
  }

  public boolean getIsTargetFound() {
    double o = ta;
    if (o <= 0.05) {
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
    if (tx < 6.5 || tx > 11.5) {
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
  public Pose2d getPosition() {
      Pose2d T_robot_to_tag = new Pose2d(targetPoseZ, -targetPoseX, new Rotation2d(-Math.atan2(targetPoseX, targetPoseZ)));
      Pose2d T_field_to_tag = new Pose2d(-.04, 5.56, new Rotation2d(0));

      //Pose2d T_field_to_robot = T_field_to_tag.transformBy(T_robot_to_tag);

      return null;
  }


  public Command turnRobot(SwerveSubsystem d){
    return new FunctionalCommand(
    () -> {

    }, 
    () -> {
      setpipeline(0);
      //YAW
      double pos = targetPoseYaw;
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

  public Command limelightScore(ArmSubsystem a, ShooterSubsystem s){
    return new FunctionalCommand(
    () -> {
      a.goHome();
      
      setpipeline(0);
    }, 
    () -> {
      SmartDashboard.putNumber("POS Z", targetPoseZ);
   
      a.moveArmForward();
    },
    (__unused) -> {
     
    },
    () -> {
      //6ft-6inch -> 1.9812 Meters
      //targetPoseZ returns distance away from april tag
      if(targetPoseZ == 0) return false;
      //Robot Length from Limelight to the shooter on the ground
      //Ours is measured at 1.9431
      targetPoseZ += 0;
      double theta = Math.atan2(1.9812, targetPoseZ);
      double armPos = a.getPosition();
      //something to tune our shooting
      double modifier = 0;
      return (armPos + modifier) >= theta;
    },
    this, a
    );
  }

   
 

  
  public boolean tagAlign() {
    if(Math.abs(targetPoseX) <= 0.075) {
      return true;
    }
    return false;
  }

}
