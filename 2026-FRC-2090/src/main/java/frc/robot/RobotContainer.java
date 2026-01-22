// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods.
 */
public class RobotContainer {
  public static final CommandXboxController driverXbox =
      new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final SingleMotorTest motorTest = new SingleMotorTest(Constants.TestMotorConstants.MOTOR_ID);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  private void configureBindings() {
    Command runMotor = new StartEndCommand(
        () -> motorTest.setPower(Constants.TestMotorConstants.TEST_POWER),
        motorTest::stop,
        motorTest);

    driverXbox.a().whileTrue(runMotor);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean enabled) {
    motorTest.setMotorBrake(enabled);
  }
}
