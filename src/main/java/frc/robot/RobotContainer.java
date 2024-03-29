// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.Auton.Intake;
import frc.robot.commands.Auton.PreShooter;
import frc.robot.commands.Teleop.AmpPositionCommand;
import frc.robot.commands.Teleop.DriveController;
import frc.robot.commands.Teleop.ShootCommand;
import frc.robot.commands.Auton.Shooter;
import frc.robot.commands.Auton.Home;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {


  public SwerveSubsystem drive;
  protected Pigeon2 gyro;
  protected Limelight lime;
  protected IntakeSubsystem intake;
  protected ShooterSubsystem shooter;
  protected ArmSubsystem arm;
  protected ClimberSubsystem climber;
  protected CommandXboxController armController;
  protected CommandXboxController driveController;
  private SendableChooser<PathPlannerPath> pathChooser;
  private SendableChooser<String> pathAutonChooser;


  public RobotContainer() {
   
    arm = new ArmSubsystem();
   // lime = new Limelight();
    drive = new SwerveSubsystem();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    climber = new ClimberSubsystem();
    armController = new CommandXboxController(Constants.Gamepad.Controller.ARM);
    driveController = new CommandXboxController(Constants.Gamepad.Controller.DRIVE);

    configureBindings();

    NamedCommands.registerCommand("Shoot", new Shooter(shooter, intake, arm));
    NamedCommands.registerCommand("Intake", new Intake(shooter, intake, arm));
    NamedCommands.registerCommand("StopIntake", intake.stopIntaking());
    NamedCommands.registerCommand("Home", new Home(shooter, intake, arm));
    NamedCommands.registerCommand("StopDriving", drive.stopDriving());
    NamedCommands.registerCommand("MiddlePreShoot", new PreShooter(shooter, intake, arm, Constants.Position.MainArm.AUTONLMIDDLESHOOT));
    NamedCommands.registerCommand("SidePreShoot", new PreShooter(shooter, intake, arm, Constants.Position.MainArm.AUTONSIDESHOOT));
   
  
    this.pathChooser = new SendableChooser<PathPlannerPath>();
    
    this.pathAutonChooser = new SendableChooser<String>();
    this.pathAutonChooser.setDefaultOption("Blue-Left", "Blue-Left");
    this.pathAutonChooser.addOption("Blue-Middle", "Blue-Middle");
    this.pathAutonChooser.addOption("Blue-Right", "Blue-Right");
    this.pathAutonChooser.addOption("Red-Left", "Blue-Right");
    this.pathAutonChooser.addOption("Red-Middle", "Blue-Middle");
    this.pathAutonChooser.addOption("Red-Right", "Blue-Left");

    this.pathChooser.setDefaultOption("1 Meter Without Spin", PathPlannerPath.fromPathFile("1 Meter Without Spin"));
    this.pathChooser.addOption("1 Meter Without Spin Y", PathPlannerPath.fromPathFile("1 Meter Without Spin"));
    this.pathChooser.addOption("3 Meter Without Spin", PathPlannerPath.fromPathFile("3 Meter Without Spin"));
    this.pathChooser.addOption("1 Meter - 90 Degree Spin", PathPlannerPath.fromPathFile("1 Meter - 90 Degree Spin"));
    this.pathChooser.addOption("3 Meter - 90 Degree Spin", PathPlannerPath.fromPathFile("3 Meter - 90 Degree Spin"));
    this.pathChooser.addOption("1 Meter - 180 Degree Spin", PathPlannerPath.fromPathFile("1 Meter - 180 Degree Spin"));
    this.pathChooser.addOption("3 Meter - 180 Degree Spin", PathPlannerPath.fromPathFile("3 Meter - 180 Degree Spin"));


    SmartDashboard.putData("Path Choices", pathChooser);
    SmartDashboard.putData("Auton Choices", pathAutonChooser);
  }

  private void configureBindings() {
    intake.setDefaultCommand(intake.stopIntaking());
    //shooter.setDefaultCommand(shooter.stopShooting());
    //shooter.setDefaultCommand();
    arm.setDefaultCommand(arm.maintainArm());
    climber.setDefaultCommand(climber.maintainClimberPosCommand());

    armController.x().whileTrue(intake.startIntake());
    armController.x().onFalse(intake.stopIntaking());

    armController.x().onTrue(shooter.setShooterBrake());
    armController.x().onFalse(shooter.setShooterCoast());


    armController.y().whileTrue(arm.shootTarget());
 
    armController.rightBumper().whileTrue(arm.pickupTarget());
    armController.leftBumper().whileTrue(arm.goHome());

    armController.povLeft().whileTrue(arm.ampTarget());
    
    armController.povUp()
      .and(arm::isNotAtTop)
      .whileTrue(arm.moveArmForward());

    armController.povDown()
      .and(arm::isNotAtBottom)
      .whileTrue(arm.moveArmBackwards());

    armController.back().onTrue(drive.rotate90());



    //DRIVE CONTROLLER
    driveController.leftBumper().whileTrue(climber.moveClimberUp());
    driveController.rightBumper().whileTrue(climber.moveClimberDown());

    //Moving Arm Positions
    driveController.x().onTrue(arm.shootTarget());
    driveController.a().onTrue(arm.pickupTarget());

    //Reverse Intake
    driveController.b().whileTrue(intake.reverseIntake());
    driveController.b().onFalse(intake.stopIntaking());

    //Amp Shooting
    driveController.y().whileTrue(new AmpPositionCommand(shooter, arm).andThen(shooter.teleopShootCommand()).alongWith(intake.startIntake()));
    driveController.y().onFalse(shooter.stopShooting());

    //Intaking 
    driveController.a()
      .whileTrue(intake.startIntake().alongWith(shooter.setShooterBrake()));
    driveController.a().onFalse(new ReverseIntakeCommand(intake).andThen(intake.stopIntaking()));
    
  
    //Teleop PreShooter
    driveController.leftTrigger().onTrue(shooter.teleopShootCommand());
    driveController.leftTrigger().onFalse(shooter.stopShooting());
  

    //Teleop Shooting 
    driveController.rightTrigger().whileTrue(new ShootCommand(shooter, intake, arm));
    driveController.rightTrigger().onFalse(shooter.stopShooting());

    //Reset Gyro
    driveController.back().whileTrue(drive.resetGyroCommand());


    driveController.povUp().onTrue(
      new DriveController(drive, driveController::getRightX, driveController::getLeftY, driveController::getLeftX, 1));
    driveController.povDown().onTrue(
      new DriveController(drive, driveController::getRightX, driveController::getLeftY, driveController::getLeftX, 4)
    );

    driveController.start()
      .and(arm::isNotAtBottom)
      .whileTrue(arm.moveArmBackwards());

    //Driving with Joysticks default command
    drive.setDefaultCommand(
        new DriveController(drive, driveController::getRightX, driveController::getLeftY, driveController::getLeftX, 4));
  }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(this.pathAutonChooser.getSelected());
   //return AutoBuilder.followPathWithEvents(pathChooser.getSelected());
  }


}
