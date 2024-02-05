package frc.robot.SwerveClasses;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveOdometry {
  SwerveDriveOdometry swerveOdometry;
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
            Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[FR] =
        new Translation2d(
            Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[BL] =
        new Translation2d(
            -Constants.Measurement.TRACK_WIDTH / 2.0, Constants.Measurement.WHEELBASE / 2.0);
    vectorKinematics[BR] =
        new Translation2d(
            -Constants.Measurement.TRACK_WIDTH / 2.0, -Constants.Measurement.WHEELBASE / 2.0);

    swerveKinematics =
        new SwerveDriveKinematics(
            vectorKinematics[FL], vectorKinematics[FR], vectorKinematics[BL], vectorKinematics[BR]);

    swerveOdometry =
        new SwerveDriveOdometry(
            swerveKinematics,
            gyro.getRotation2d().times(1),
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
    ;
  }

  public void update() {
    swerveOdometry.update(
        gyro.getRotation2d().times(1),
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

  public Pose2d position() {
    double x = swerveOdometry.getPoseMeters().getX();
    double y = swerveOdometry.getPoseMeters().getY();
    
    Rotation2d rotation = swerveOdometry.getPoseMeters().getRotation();
    //rotation.times(360);
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
        gyro.getRotation2d().times(1),
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
        gyro.getRotation2d().times(1),
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
