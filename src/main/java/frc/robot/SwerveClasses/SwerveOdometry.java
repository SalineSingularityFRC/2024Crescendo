package frc.robot.SwerveClasses;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PidGains.Limelight;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveOdometry {
  SwerveDrivePoseEstimator swerveOdometry;
  public Pigeon2 gyro;

  private final int FL = 0;
  private final int FR = 1;
  private final int BL = 2;
  private final int BR = 3;

  private final SwerveDriveKinematics swerveKinematics;

  private SwerveModule[] swerveModules = new SwerveModule[4];
  private final Translation2d[] vectorKinematics = new Translation2d[4];
  private SwerveSubsystem subsystem;

  public SwerveOdometry(SwerveSubsystem subsystem) {
    this.subsystem = subsystem;
    gyro = subsystem.gyro;
    vectorKinematics[FL] =
        new Translation2d(
            Constants.Measurement.WHEEL_BASE / 2, Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[FR] =
        new Translation2d(
            Constants.Measurement.WHEEL_BASE / 2, -Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[BL] =
        new Translation2d(
            -Constants.Measurement.WHEEL_BASE / 2, Constants.Measurement.TRACK_WIDTH / 2);
    vectorKinematics[BR] =
        new Translation2d(
            -Constants.Measurement.WHEEL_BASE / 2, -Constants.Measurement.TRACK_WIDTH / 2);

    swerveKinematics =
        new SwerveDriveKinematics(
            vectorKinematics[FL], vectorKinematics[FR], vectorKinematics[BL], vectorKinematics[BR]);

    swerveOdometry =
        new SwerveDrivePoseEstimator(
            swerveKinematics,
            new Rotation2d(subsystem.getRobotAngle()),
            new SwerveModulePosition[] {
              new SwerveModulePosition(
                  subsystem.swerveModules[FL].getPosition(),
                  new Rotation2d(subsystem.swerveModules[FL].getEncoderPosition())),
              new SwerveModulePosition(
                  subsystem.swerveModules[FR].getPosition(),
                  new Rotation2d(subsystem.swerveModules[FR].getEncoderPosition())),
              new SwerveModulePosition(
                  subsystem.swerveModules[BL].getPosition(),
                  new Rotation2d(subsystem.swerveModules[BL].getEncoderPosition())),
              new SwerveModulePosition(
                  subsystem.swerveModules[BR].getPosition(),
                  new Rotation2d(subsystem.swerveModules[BR].getEncoderPosition())),
            },
            new Pose2d(0, 0, new Rotation2d()));
  }

  public void update() {
    swerveOdometry.update(
        new Rotation2d(subsystem.getRobotAngle()),
        new SwerveModulePosition[] {
          new SwerveModulePosition(
              subsystem.swerveModules[FL].getPosition(),
              new Rotation2d(subsystem.swerveModules[FL].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[FR].getPosition(),
              new Rotation2d(subsystem.swerveModules[FR].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[BL].getPosition(),
              new Rotation2d(subsystem.swerveModules[BL].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[BR].getPosition(),
              new Rotation2d(subsystem.swerveModules[BR].getEncoderPosition())),
        });

     
  }

  public void visionUpdate(){
       // Compute the robot's field-relative position exclusively from vision measurements.
        PoseEstimate limelightPosEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
       
        // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
        Pose2d visionMeasurement2d = limelightPosEstimate.pose;
        SmartDashboard.putNumber("LimeLight Estimate X", visionMeasurement2d.getX());
        SmartDashboard.putNumber("LimeLight Estimate Y", visionMeasurement2d.getY());
        swerveOdometry.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp() - (limelightPosEstimate.latency/1000.0));
  }

   

  public Pose2d position() {
    
    double x = swerveOdometry.getEstimatedPosition().getX();
    double y = swerveOdometry.getEstimatedPosition().getY();
    
    Rotation2d rotation = swerveOdometry.getEstimatedPosition().getRotation();
    //rotation.times(360);
    SmartDashboard.putNumber("Odometry X" , x);
    SmartDashboard.putNumber("Odometry Y" , y);

    return new Pose2d(new Translation2d(x, y), rotation);
  }

  public double getRotation() {
    return position().getRotation().getDegrees();
  }

  public double getX() {
    return position().getX();
  }

public double getY() {
    return position().getY();
  }

  public void resetPosition() {
    swerveOdometry.resetPosition(
        new Rotation2d(subsystem.getRobotAngle()),
        new SwerveModulePosition[] {
          new SwerveModulePosition(
              subsystem.swerveModules[FL].getPosition(),
              new Rotation2d(subsystem.swerveModules[FL].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[FR].getPosition(),
              new Rotation2d(subsystem.swerveModules[FR].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[BL].getPosition(),
              new Rotation2d(subsystem.swerveModules[BL].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[BR].getPosition(),
              new Rotation2d(subsystem.swerveModules[BR].getEncoderPosition())),
        },
        new Pose2d(0, 0, new Rotation2d()));
  }

    public void setPosition(Pose2d pos) {
    swerveOdometry.resetPosition(
        new Rotation2d(subsystem.getRobotAngle()),
        new SwerveModulePosition[] {
          new SwerveModulePosition(
              subsystem.swerveModules[FL].getPosition(),
              new Rotation2d(subsystem.swerveModules[FL].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[FR].getPosition(),
              new Rotation2d(subsystem.swerveModules[FR].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[BL].getPosition(),
              new Rotation2d(subsystem.swerveModules[BL].getEncoderPosition())),
          new SwerveModulePosition(
              subsystem.swerveModules[BR].getPosition(),
              new Rotation2d(subsystem.swerveModules[BR].getEncoderPosition())),
        },
        pos);
  }
}
