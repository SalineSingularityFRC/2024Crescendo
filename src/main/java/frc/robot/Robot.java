// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private SwerveSubsystem robotSubsystem;

  private Gamepad teleopDrive;

  private ArmSubsystem arm;
  private Limelight limelight;
  private LightSensor cubelightSensor;
  private LightSensor conelightSensor;
  public static SwerveOdometry odometry;
  private final int FL = 0;

  @Override
  public void robotInit() {

    // Required to allow power to the switchable port on the power distrubution hub and allow sensor
    // to use max power
    PowerDistribution PD = new PowerDistribution();
    PD.setSwitchableChannel(true);

    robotSubsystem = new SwerveSubsystem();


    teleopDrive = new Gamepad(Constants.Gamepad.Controller.DRIVE, Constants.Gamepad.Controller.ARM);
    odometry = new SwerveOdometry(robotSubsystem);

    arm = new ArmSubsystem(false, true);

    limelight = new Limelight();
    
    m_robotContainer =
        new RobotContainer(
            arm,
            robotSubsystem,
            robotSubsystem.gyro,
            limelight,
            cubelightSensor,
            conelightSensor,
            odometry);
    robotSubsystem.resetGyro();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    odometry.update();
    SmartDashboard.putNumber("Odometry Rotation", odometry.getRotation());
    SmartDashboard.putNumber("Odometry X", odometry.getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getY());

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //Needs testing, may fix issue.
    //robotSubsystem.drive(new SwerveSubsystem.SwerveRequest(0, 0, 0), true);
    odometry.resetPosition();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    robotSubsystem.setBrakeMode();

    if (m_autonomousCommand != null) {

      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    robotSubsystem.setCoastMode();
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive.swerveDrive(
        robotSubsystem, limelight, arm, cubelightSensor, conelightSensor, odometry);
    teleopDrive.arm(arm);
    
    SmartDashboard.putNumber("tx", limelight.tx.getDouble(0));
    SmartDashboard.putNumber("ty", limelight.ty.getDouble(0));
    SmartDashboard.putNumber("ta", limelight.ta.getDouble(0));
    //SmartDashboard.putNumber("tl", limelight.tl.getDouble(0));
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
