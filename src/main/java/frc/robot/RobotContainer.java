// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AmpPositionCommand;
import frc.robot.commands.AutonHomeCommand;
import frc.robot.commands.AutonIntakeCommand;
import frc.robot.commands.AutonMiddlePreShooter;
import frc.robot.commands.AutonPreShootCommand;
import frc.robot.commands.AutonShooterCommand;
import frc.robot.commands.AutonSidePreShooter;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.DriveController;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StartIntakeCommand;
import frc.robot.commands.StartShootCommand;
import frc.robot.commands.TeleopShootCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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

    NamedCommands.registerCommand("Shoot", new AutonShooterCommand(shooter, intake, arm));
    NamedCommands.registerCommand("Intake", new AutonIntakeCommand(shooter, intake, arm));
    NamedCommands.registerCommand("StopIntake", intake.stopIntaking());
    NamedCommands.registerCommand("Home", new AutonHomeCommand(shooter, intake, arm));
    NamedCommands.registerCommand("StopDriving", drive.stopDriving());
    NamedCommands.registerCommand("MiddlePreShoot", new AutonMiddlePreShooter(shooter, intake, arm));
    NamedCommands.registerCommand("SidePreShoot", new AutonSidePreShooter(shooter, intake, arm));
    
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


    //this.pathAutonChooser.setDefaultOption("3 - METER",  new PathPlannerAuto("New Auto"));

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
    armController.x().onTrue(shooter.setShooterBrake());
    armController.x().onFalse(shooter.setShooterCoast());

    armController.a().whileTrue(intake.reverseIntake());
       armController.b().whileTrue(new ShootCommand(shooter, intake, arm, drive));

    
    armController.y().whileTrue(arm.shootTarget());
 
    armController.rightBumper().whileTrue(arm.pickupTarget());
    armController.leftBumper().whileTrue(arm.goHome());

    armController.povLeft().whileTrue(arm.ampTarget());

    // driveController.povUp().whileTrue(
      // new DriveController(drive, driveController::getRightX, driveController::getLeftY, driveController::getLeftX, 0.25));
    armController.povUp()
      .and(arm::isNotAtTop)
      .whileTrue(arm.moveArmForward());

    armController.povDown()
      .and(arm::isNotAtBottom)
      .whileTrue(arm.moveArmBackwards());

    armController.back().onTrue(drive.rotate90());



    //DRIVE CONTROLLER
    driveController.leftBumper().whileTrue(new ClimberUpCommand(climber, arm));
    driveController.rightBumper().whileTrue(new ClimberDownCommand(climber, arm));

    driveController.x().onTrue(arm.shootTarget());

     driveController.a().onTrue(arm.pickupTarget());
    driveController.a()
      .whileTrue(intake.startIntake().alongWith(shooter.setShooterBrake()));
    
    driveController.b().whileTrue(intake.reverseIntake());
    driveController.y().whileTrue(new AmpPositionCommand(shooter, arm).andThen(shooter.startShooting()));

    driveController.leftTrigger().whileTrue(intake.startIntake());
    driveController.leftTrigger().onTrue(arm.pickupTarget());
  
    driveController.rightTrigger().whileTrue(new TeleopShootCommand(shooter, intake, arm));
    
    driveController.rightTrigger().onFalse(shooter.stopShooting());
    driveController.back().whileTrue(drive.resetGyroCommand());

    driveController.start()
      .and(arm::isNotAtTop)
      .whileTrue(arm.moveArmForward());

    driveController.leftTrigger()
      .and(arm::isNotAtBottom)
      .whileTrue(arm.moveArmBackwards());

    drive.setDefaultCommand(
        new DriveController(drive, driveController::getRightX, driveController::getLeftY, driveController::getLeftX, 4));
    //armController.y().whileTrue(lime.scoreRight(drive));
  }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(this.pathAutonChooser.getSelected());
   //return AutoBuilder.followPathWithEvents(pathChooser.getSelected());
  }


}
