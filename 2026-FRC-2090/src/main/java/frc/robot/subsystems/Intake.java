// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final String kAppliedOutputEntry = "Intake Applied Output";
  private static final String kEncoderPositionEntry = "Intake Encoder Position";

  private static final int INTAKE_MOTOR_ID = 30;
  private static final int INTAKE_DROPDOWN_ID = 19;

  // Intake speeds (adjust as needed)
  private static final double INTAKE_SPEED = 0.6;
  private static final double OUTTAKE_SPEED = -0.3;
  private static final double SLOW_INTAKE_SPEED = 0.3;

  // Dropdown positions (motor rotations) - adjust to match your mechanism
  private static final double DROPDOWN_POS_UP = -14.63232421875;
  private static final double DROPDOWN_POS_DOWN = -11.90576171875;

  private final TalonFX motor;
  // Dropdown uses a Kraken X44 (TalonFX) and will be position-controlled
  private final TalonFX dropdownMotor;

  private final PositionVoltage dropdownPosition = new PositionVoltage(0.0);
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final TalonFXConfiguration dropdownConfig = new TalonFXConfiguration();

  public Intake() {
    motor = new TalonFX(INTAKE_MOTOR_ID);
    dropdownMotor = new TalonFX(INTAKE_DROPDOWN_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  motor.getConfigurator().apply(config);

  // Configure dropdown motor PID for position control on its own slot
  // Reduced kP to avoid oscillation; tune on robot per instructions below
  dropdownConfig.Slot0.kP = 2; // starting P
  dropdownConfig.Slot0.kI = 0.0;
  dropdownConfig.Slot0.kD = 0.0;
  dropdownConfig.Slot0.kV = 10;
  dropdownConfig.Slot0.kS = 10;
  dropdownConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  dropdownConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  dropdownMotor.getConfigurator().apply(dropdownConfig);

    motor.setPosition(0.0);
    dropdownMotor.setPosition(DROPDOWN_POS_UP); // Start with dropdown in the up position
    // Dropdown initial state: false = up, true = down
    dropdownDown = false;
  }



  // Tracks whether the dropdown is currently down
  private boolean dropdownDown = false;

  /**
   * Set the intake motor power.
   * @param power Motor power from -1.0 to 1.0
   */
  public void setPower(double power) {
    motor.setControl(dutyCycle.withOutput(MathUtil.clamp(-power, -1.0, 1.0)));
  }

  /**
   * Move the dropdown to a target position in motor rotations using closed-loop position control.
   * @param rotations target in motor rotations
   */
  public Command setDropdownPosition(double rotations) {
    // PositionVoltage expects rotations (TalonFX native units here are rotations via getPosition())
    return run(() -> dropdownMotor.setControl(dropdownPosition.withPosition(rotations)));

  }

  /**
   * Command wrapper to move dropdown to position (runOnce)
   */
  public edu.wpi.first.wpilibj2.command.Command moveDropdownTo(double rotations) {
    return runOnce(() -> setDropdownPosition(rotations)).andThen(run(() -> {}).withTimeout(0.01));
  }

  /**
   * Get dropdown current position in rotations
   */
  public double getDropdownPosition() {
    return dropdownMotor.getPosition().getValueAsDouble();
  }

  /** Toggle dropdown between up and down positions. */
  public void toggleDropdown() {
      if (!dropdownDown) {
          dropdownMotor.setControl(dropdownPosition.withPosition(DROPDOWN_POS_DOWN));
          dropdownDown = true;
      } else {
          dropdownMotor.setControl(dropdownPosition.withPosition(DROPDOWN_POS_UP));
          dropdownDown = false;
      }
  }

  /** Command wrapper to toggle dropdown state on button press. */
  public Command toggleDropdownCommand() {
    return runOnce(this::toggleDropdown);
  }

  /**
   * Stop the intake motor.
   */
  public void stop() {
    motor.setControl(dutyCycle.withOutput(0.0));
  }

  /**
   * Set the motor brake mode.
   * @param enabled true for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(kAppliedOutputEntry, motor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(kEncoderPositionEntry, motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Dropdown Position", getDropdownPosition());
  }

  // ===== Commands =====

  /**
   * Command to run the intake at the default intake speed.
   */
  public Command intakeCommand() {
    return run(() -> this.setPower(INTAKE_SPEED))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the intake at a slow speed.
   */
  public Command slowIntakeCommand() {
    return run(() -> this.setPower(SLOW_INTAKE_SPEED))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the intake in reverse (outtake).
   */
  public Command outtakeCommand() {
    return run(() -> this.setPower(OUTTAKE_SPEED))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the intake at a variable speed.
   * @param speed The speed to run at (-1.0 to 1.0)
   */
  public Command runAtSpeed(double speed) {
    return run(() -> this.setPower(speed))
        .finallyDo(() -> this.stop());
  }
}
