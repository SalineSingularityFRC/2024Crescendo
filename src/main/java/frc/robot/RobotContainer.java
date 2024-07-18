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

import frc.robot.commands.Auton.Intake;
import frc.robot.commands.Auton.PreShooter;
import frc.robot.commands.Teleop.AmpPositionCommand;
import frc.robot.commands.Teleop.DriveController;
import frc.robot.commands.Teleop.ShootCommand;
import frc.robot.commands.Auton.Shooter;
import frc.robot.commands.Auton.StopIntake;
import frc.robot.commands.Limelight.LimelightPreShoot;
import frc.robot.commands.Auton.Home;
import frc.robot.commands.IntakeParallelCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.Teleop.toSpeaker;
import frc.robot.commands.Teleop.toAmp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.SwerveClasses.*;

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
        NamedCommands.registerCommand("StopIntake", new StopIntake(shooter, intake));
        NamedCommands.registerCommand("Home", new Home(shooter, intake, arm));
        NamedCommands.registerCommand("StopDriving", drive.stopDriving());
        //NamedCommands.registerCommand("IntakeBarf", new IntakeBarf(intake, shooter));
        NamedCommands.registerCommand("StopShooting", shooter.stopShooting());

        NamedCommands.registerCommand("SpeakerSidePreShoot",
                new PreShooter(shooter, intake, arm, Constants.Position.MainArm.Speaker.SIDE));
        NamedCommands.registerCommand("SpeakerMiddlePreShoot",
                new PreShooter(shooter, intake, arm, Constants.Position.MainArm.Speaker.MIDDLE));
        NamedCommands.registerCommand("CloseNoteSidePreShoot",
                new PreShooter(shooter, intake, arm, Constants.Position.MainArm.Auton.CloseNote.SIDE));
        NamedCommands.registerCommand("WhiteLineSidePreShoot",
                new PreShooter(shooter, intake, arm, Constants.Position.MainArm.Auton.WhiteLine.SIDE));
        NamedCommands.registerCommand("WhiteLineMiddlePreShoot",
                new PreShooter(shooter, intake, arm, Constants.Position.MainArm.Auton.WhiteLine.MIDDLE));

        this.pathChooser = new SendableChooser<PathPlannerPath>();

        this.pathAutonChooser = new SendableChooser<String>();


        this.pathAutonChooser.setDefaultOption("BlueLeft-2-Note", "BlueLeft-2-Note");
        this.pathAutonChooser.addOption("BlueLeft-3-Note", "BlueLeft-3-Note");

        this.pathAutonChooser.addOption("BlueRight-2-Note-CloseNote", "BlueRight-2-Note-CloseNote");
        this.pathAutonChooser.addOption("BlueRight-3-Note-FarNote-Then-Close", "BlueRight-3-Note-FarNote-Then-Close");
        this.pathAutonChooser.addOption("BlueRight-3-Note-Close-Then-FarNote", "BlueRight-3-Note-Close-Then-FarNote");

        this.pathAutonChooser.addOption("RedLeft-2-Note-CloseNote", "BlueRight-2-Note-CloseNote");
        this.pathAutonChooser.addOption("RedLeft-3-Note-FarNote-Then-Close", "BlueRight-3-Note-FarNote-Then-Close");
        this.pathAutonChooser.addOption("RedRight-2-Note", "BlueLeft-2-Note");
        this.pathAutonChooser.addOption("RedRight-3-Note", "BlueLeft-3-Note");

        this.pathAutonChooser.addOption("Middle-2-Note", "BlueMiddle-2-Note");

        this.pathAutonChooser.addOption("BlueMiddle-3-Note-Close2&1", "BlueMiddle-3-Note-Close2&1");
        this.pathAutonChooser.addOption("BlueMiddle-3-Note-Close2&3", "BlueMiddle-3-Note-Close2&3");
        this.pathAutonChooser.addOption("RedMiddle-3-Note-Close2&3", "BlueMiddle-3-Note-Close2&1");
        this.pathAutonChooser.addOption("RedMiddle-3-Note-Close2&1", "BlueMiddle-3-Note-Close2&3");

        this.pathAutonChooser.addOption("Middle-3-Note-FarNote3", "BlueMiddle-3-Note-FarNote3");

        this.pathAutonChooser.addOption("Middle-4-Note", "BlueMiddle-4-Note");

        //For the open house
        this.pathAutonChooser.addOption("Open House Test", "Open House Test");


        // this.pathChooser.setDefaultOption("1 Meter Without Spin",
        // PathPlannerPath.fromPathFile("1 Meter"));
        // this.pathChooser.addOption("1 Meter Without Spin Y",
        // PathPlannerPath.fromPathFile("1 Meter Without Spin"));
        // this.pathChooser.addOption("3 Meter Without Spin",
        // PathPlannerPath.fromPathFile("3 Meter Without Spin"));
        // this.pathChooser.addOption("1 Meter - 90 Degree Spin",
        // PathPlannerPath.fromPathFile("1 Meter - 90 Degree Spin"));
        // this.pathChooser.addOption("3 Meter - 90 Degree Spin",
        // PathPlannerPath.fromPathFile("3 Meter - 90 Degree Spin"));
        // this.pathChooser.addOption("1 Meter - 180 Degree Spin",
        // PathPlannerPath.fromPathFile("1 Meter - 180 Degree Spin"));
        // this.pathChooser.addOption("3 Meter - 180 Degree Spin",
        // PathPlannerPath.fromPathFile("3 Meter - 180 Degree Spin"));

        SmartDashboard.putData("Path Choices", pathChooser);
        SmartDashboard.putData("Auton Choices", pathAutonChooser);
    }

    private void configureBindings() {
        intake.setDefaultCommand(intake.stopIntaking());
        // shooter.setDefaultCommand(shooter.stopShooting());
        // shooter.setDefaultCommand();
        arm.setDefaultCommand(arm.maintainArm());
        climber.setDefaultCommand(climber.maintainClimberPosCommand());

        //Arm Controller
        //Intake for arm controller
        armController.a().onTrue(shooter.setShooterBrake());
        armController.a()
                .whileTrue(new IntakeParallelCommand(shooter, intake, 0.15).alongWith(arm.pickupTarget()));
        armController.a().onFalse(new ReverseIntakeCommand(intake).andThen(intake.stopIntaking()).andThen(shooter.stopShooting()).andThen(shooter.setShooterCoast()));
        //armController.a().onFalse(shooter.stopShooting());

        // Home for arm controller (one button press)
        //armController.leftBumper().whileTrue(arm.goHome());
        
        // Amp Position
        armController.y().whileTrue(
                new AmpPositionCommand(shooter, arm));

        // Amp Shooting
        //armController.rightBumper().whileTrue(shooter.ampShootCommand()
        //                .alongWith(intake.startIntake()));
        //armController.rightBumper().onFalse(shooter.stopShooting());

        // Manual arm movement
        armController.povUp()
                .and(arm::isNotAtTop)
                .whileTrue(arm.moveArmForward());
        armController.povDown()
                .and(arm::isNotAtBottom)
                .whileTrue(arm.moveArmBackwards());

        // Moving Arm Positions
        armController.x().whileTrue(arm.shootTarget());

        // Reverse Intake
        armController.b().whileTrue(intake.reverseIntake().alongWith(shooter.reverseShooter(5.0)));
        armController.b().onFalse(intake.stopIntaking().alongWith(shooter.stopShooting()));


        // DRIVE CONTROLLER

        // Climber Controller
        // Not used in the most recent comp
        // driveController.leftBumper().whileTrue(climber.moveClimberUp());
        // driveController.rightBumper().whileTrue(climber.moveClimberDown());

        // Teleop PreShooter
        armController.leftTrigger().onTrue(shooter.teleopShootCommand());
        armController.leftTrigger().onFalse(shooter.stopShooting());

        // Teleop Shooting
        armController.rightTrigger().whileTrue(new ShootCommand(shooter, intake, arm));// .alongWith(drive.xMode()));
        armController.rightTrigger().onFalse(shooter.stopShooting());

        // Reset Gyro
        driveController.x().whileTrue(drive.resetGyroCommand());

        // Limelight drive to x distance to speaker
        // driveController.b().whileTrue(
        //         new toSpeaker(drive, lime)
        // );

        // Needs to be tested
        // Will first start up pre shooter and then go to the nearest distance 
        // we can shoot from. Right after, it will shoot from that position.
        // driveController.y().whileTrue(
        //         (new LimelightPreShoot(shooter, drive, arm, lime, intake))
        //         .andThen(new ShootCommand(shooter, intake, arm))
        // );
        // driveController.y().onFalse(
        //         shooter.stopShooting()
        // );

        // // Limelight drive to amp
        // // Not tested
        // driveController.povDown().whileTrue(
        //         new toAmp(drive, lime)
        // );

        // (Should be automatic)
        driveController.povRight().onTrue(drive.xMode());

        // Driving with Joysticks default command scaled to x^2
        drive.setDefaultCommand(
                new DriveController(drive, () -> {
                        if (driveController.getRightX() < 0) {
                            return -1.0 * driveController.getRightX() * driveController.getRightX();
                        }

                        return driveController.getRightX() * driveController.getRightX();
                }, () -> {
                        if (driveController.getLeftY() < 0) {
                            return -1.0 * driveController.getLeftY() * driveController.getLeftY();
                        }

                        return driveController.getLeftY() * driveController.getLeftY();
                }, () -> {
                        if (driveController.getLeftX() < 0) {
                                return -1.0 * driveController.getLeftX() * driveController.getLeftX();
                        }

                        return driveController.getLeftX() * driveController.getLeftX();
                },
                        4.0));
        }

    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI

        // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        return new PathPlannerAuto(this.pathAutonChooser.getSelected());

        //PathPlannerPath path = PathPlannerPath.fromPathFile("1");

        // // Create a path following command using AutoBuilder. This will also trigger
        // event markers.
        //return AutoBuilder.followPath(path);

     }

}
