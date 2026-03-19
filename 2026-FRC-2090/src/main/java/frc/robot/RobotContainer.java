// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.jar.Attributes.Name;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.ShooterTransferCommand;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/falcon"));

  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Hood hood = new Hood();
  private final Transfer transfer = new Transfer();

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driverXbox.getLeftY(),
      () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math! bro what is this ai comment
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driverXbox.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driverXbox.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure PathPlanner AutoBuilder with the swerve drivetrain
    SendableChooser<Command> chooser;
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          drivebase::getPose,                                    // Pose supplier
          drivebase::resetOdometry,                             // Reset odometry
          drivebase::getRobotVelocity,                          // Robot-relative ChassisSpeeds
          (speeds, feedforwards) -> drivebase.drive(speeds),    // Drive method
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),                  // Translation PID
              new PIDConstants(5.0, 0.0, 0.0)                   // Rotation PID
          ),
          config,
          () -> {
            // Flip paths for red alliance
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          drivebase
      );
        NamedCommands.registerCommand("RunIntake", intake.intakeCommand());
        NamedCommands.registerCommand("DropdownIntake", intake.setDropdownPosition(-11.90576171875));
        NamedCommands.registerCommand("SpinUpShooter", shooter.runAtRPM(4000));
        NamedCommands.registerCommand("Shoot", transfer.transferCommand().withTimeout(0.75));
        
        // these dont exist because we never turn them off >:)
        // NamedCommands.registerCommand("StopShooter", shooter.stopCommand());
        // NamedCommands.registerCommand("StopIntake", intake.stopCommand());

      // Load all .auto files from deploy/pathplanner/autos/ automatically
      // "default_auto" becomes the default selection
      chooser = AutoBuilder.buildAutoChooser("Copy of Copy of pass_auto");
    } catch (Exception e) {
      System.err.println("Failed to configure PathPlanner AutoBuilder: " + e.getMessage());
      chooser = new SendableChooser<>();
      chooser.setDefaultOption("Do Nothing", Commands.runOnce(drivebase::zeroGyroWithAlliance));
    }
    autoChooser = chooser;
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /** 
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command transferShooterCommand = new ShooterTransferCommand(shooter, transfer);

   driverXbox.rightTrigger().whileTrue(hood.variableHoodCommand(() -> driverXbox.getRightTriggerAxis()));
   driverXbox.leftTrigger().whileTrue(hood.variableHoodCommand(() -> -driverXbox.getLeftTriggerAxis()));
   
   // Hood position presets via D-pad
   driverXbox.povUp().onTrue(new InstantCommand(() -> hood.setHoodUp(), hood));
   driverXbox.povRight().onTrue(new InstantCommand(() -> hood.setHoodMiddle(), hood));
   driverXbox.povDown().onTrue(new InstantCommand(() -> hood.setHoodDown(), hood));

  // Toggle intake dropdown & transfer state with the A button
   driverXbox.a().onTrue(intake.toggleDropdownCommand());

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);

    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
          () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); 

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.none());
      // driverXbox.rightBumper().onTrue(Commands.none());
    } else {
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(intake.intakeCommand());
      driverXbox.rightBumper().whileTrue(intake.outtakeCommand());
      driverXbox.y().whileTrue(transferShooterCommand);
      driverXbox.x().onTrue(shooter.addRPM());
      driverXbox.b().whileTrue(transfer.transferReverseCommand());
      //driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}