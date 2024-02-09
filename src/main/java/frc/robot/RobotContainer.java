// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // private BlueCenterCommand blueCenterCommand;
  // private RedCenterCommand redCenterCommand;
  // private LeftSideCommand leftSideCommand;
  // private RightSideCommand rightSideCommand;
  // private SwerveCommand swerveCommand;
  // protected ClawPneumatics clawPneumatics;
  protected SwerveSubsystem drive;
  protected ArmSubsystem arm;
  protected Pigeon2 gyro;
  protected Limelight lime;
  protected LightSensor cubeSensor;
  protected LightSensor coneSensor;
  private SendableChooser<Command> autonChooser;

  public RobotContainer(
      ShooterSubsystem newArm) {
    configureBindings();

    // this.drive = drive;
    // this.arm = arm;
    // this.gyro = gyro;
    // this.lime = lime;
    // this.cubeSensor = cubeSensor;

    // this.blueCenterCommand = new BlueCenterCommand(arm, clawPneumatics, drive, gyro, odometry);
    // this.redCenterCommand = new RedCenterCommand(arm, clawPneumatics, drive, gyro, odometry);

    // this.swerveCommand = new SwerveCommand(arm, clawPneumatics, drive, gyro, odometry);
    // this.leftSideCommand = new LeftSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor);
    // this.rightSideCommand =
    //     new RightSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor, odometry);

    // this.autonChooser = new SendableChooser<Command>();
    // this.autonChooser.setDefaultOption("BlueCenter", blueCenterCommand);
    // this.autonChooser.addOption("RedCenter", redCenterCommand);
    // this.autonChooser.addOption("LeftSide", leftSideCommand);
    // this.autonChooser.addOption("RightSide", rightSideCommand);
    // this.autonChooser.addOption("SwerveDriveCommand", swerveCommand);

    // SmartDashboard.putData("Auton Choices", autonChooser);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }

  // public Command getAutonomousCommand() {
  //   // Load the path you want to follow using its name in the GUI
  //   PathPlannerPath path = PathPlannerPath.fromPathFile("Left");

  //   // Create a path following command using AutoBuilder. This will also trigger event markers.
  //   return AutoBuilder.followPathWithEvents(path);
  // }
}
