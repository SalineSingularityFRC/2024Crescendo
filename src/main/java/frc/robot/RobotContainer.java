// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveController;
import frc.robot.commands.ShooterController;
import frc.robot.commands.IntakeController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  protected Pigeon2 gyro;
  protected Limelight lime;
  private SendableChooser<Command> autonChooser;
  private WaitCommand shooWaitCommand;
  protected IntakeSubsystem intake;
  protected ShooterSubsystem shooter;
  protected ArmSubsystem arm;
  protected CommandXboxController armController;
  protected CommandXboxController driveController;
  private SendableChooser<PathPlannerPath> pathChooser;
  public RobotContainer() {
   
    arm = new ArmSubsystem();
    lime = new Limelight();
    drive = new SwerveSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    
    armController = new CommandXboxController(Constants.Gamepad.Controller.ARM);
    driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);
   


    configureBindings();

    // this.arm = arm;
    // this.gyro = gyro;

    // this.cubeSensor = cubeSensor;

    // this.blueCenterCommand = new BlueCenterCommand(arm, clawPneumatics, drive,
    // gyro, odometry);
    // this.redCenterCommand = new RedCenterCommand(arm, clawPneumatics, drive,
    // gyro, odometry);

    // this.swerveCommand = new SwerveCommand(arm, clawPneumatics, drive, gyro,
    // odometry);
    // this.leftSideCommand = new LeftSideCommand(arm, clawPneumatics, drive, gyro,
    // lime, cubeSensor);
    // this.rightSideCommand =
    // new RightSideCommand(arm, clawPneumatics, drive, gyro, lime, cubeSensor,
    // odometry);

    //NamedCommands.registerCommand("autoBalance", drive.autoBalanceCommand());

    this.pathChooser = new SendableChooser<PathPlannerPath>();
    this.pathChooser.setDefaultOption("1 Meter Without Spin", PathPlannerPath.fromPathFile("1 Meter Without Spin"));
    this.pathChooser.addOption("3 Meter Without Spin", PathPlannerPath.fromPathFile("3 Meter Without Spin"));
    this.pathChooser.addOption("1 Meter - 90 Degree Spin", PathPlannerPath.fromPathFile("1 Meter - 90 Degree Spin"));
    this.pathChooser.addOption("3 Meter - 90 Degree Spin", PathPlannerPath.fromPathFile("3 Meter - 90 Degree Spin"));
    this.pathChooser.addOption("1 Meter - 180 Degree Spin", PathPlannerPath.fromPathFile("1 Meter - 180 Degree Spin"));
    this.pathChooser.addOption("3 Meter - 180 Degree Spin", PathPlannerPath.fromPathFile("3 Meter - 180 Degree Spin"));

    SmartDashboard.putData("Path Choices", pathChooser);
  }

  private void configureBindings() {
    intake.setDefaultCommand(intake.stopIntaking());
    shooter.setDefaultCommand(shooter.stopShooting());
    arm.setDefaultCommand(arm.stopArm());
    armController.a().onTrue(arm.pickupTarget());
    armController.x().whileTrue(intake.reverseIntake());
    armController.b().onTrue(arm.ampTarget());
    armController.y().onTrue(arm.shootTarget());
    
    armController.leftTrigger().whileTrue(intake.startIntake());
    armController.rightTrigger().whileTrue(shooter.startShooting());

    armController.povUp().whileTrue(arm.moveArmForward());
    armController.povDown().whileTrue(arm.moveArmBackwards());
    drive.setDefaultCommand(
        new DriveController(drive, driveController::getRightX, driveController::getLeftX, driveController::getLeftY));
    armController.rightBumper().whileTrue(lime.scoreRight(drive));
  }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPathWithEvents(pathChooser.getSelected());
  }

  // public Command getAutonomousCommand() {
  // // Load the path you want to follow using its name in the GUI
  // PathPlannerPath path = PathPlannerPath.fromPathFile("Left");

  // // Create a path following command using AutoBuilder. This will also trigger
  // event markers.
  // return AutoBuilder.followPathWithEvents(path);
  // }
}
