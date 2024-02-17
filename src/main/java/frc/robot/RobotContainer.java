// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SwerveClasses.SwerveOdometry;
import frc.robot.commands.DriveController;
import frc.robot.subsystems.ArmSubsystem;
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
  protected SendableChooser<PathPlannerPath> pathChooser;
  protected SendableChooser<Command> autoChooser;
  protected CommandXboxController driveController;

  public RobotContainer(
      ArmSubsystem arm,
      SwerveSubsystem drive,
      Pigeon2 gyro,
      Limelight lime,
      LightSensor cubeSensor,
      LightSensor coneSensor,
      SwerveOdometry odometry) {
    configureBindings();

    this.drive = drive;
    this.arm = arm;
    this.gyro = gyro;
    this.lime = lime;
    this.cubeSensor = cubeSensor;
    pathChooser = new SendableChooser<PathPlannerPath>();
    autoChooser = AutoBuilder.buildAutoChooser("Auto");
    driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE); 

    // this.blueCenterCommand = new BlueCenterCommand(arm, clawPneumatics, drive, gyro, odometry);
    // this.redCenterCommand = new RedCenterCommand(arm, clawPneumatics, drive, gyro, odometry);

    // this.swerveCommand = new SwerveCommand(arm, clawPneumatics, drive, gyro, odometry);
    // this.leftSideCommand = new LeftSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor);
    // this.rightSideCommand =
    //     new RightSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor, odometry);

    NamedCommands.registerCommand("Driving Infinitely", new DriveController(drive, driveController::getRightX, driveController::getLeftX, driveController::getLeftY));

    this.pathChooser.setDefaultOption("1 Meter Without Spin", PathPlannerPath.fromPathFile("1 Meter Without Spin"));
    this.pathChooser.addOption("3 Meter Without Spin", PathPlannerPath.fromPathFile("3 Meter Without Spin"));
    this.pathChooser.addOption("1 Meter - 90 Degree Spin", PathPlannerPath.fromPathFile("1 Meter - 90 Degree Spin"));
    this.pathChooser.addOption("3 Meter - 90 Degree Spin", PathPlannerPath.fromPathFile("3 Meter - 90 Degree Spin"));
    this.pathChooser.addOption("1 Meter - 180 Degree Spin", PathPlannerPath.fromPathFile("1 Meter - 180 Degree Spin"));
    this.pathChooser.addOption("3 Meter - 180 Degree Spin", PathPlannerPath.fromPathFile("3 Meter - 180 Degree Spin"));

    SmartDashboard.putData("Path Choices", pathChooser);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {}

  // public Command getAutonomousCommand() {
  //   return this.blueCenterCommand;
  //   //return autonChooser.getSelected();
  // }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    // return AutoBuilder.followPathWithEvents(pathChooser.getSelected());
    return new PathPlannerAuto("Auto");
  }
}
