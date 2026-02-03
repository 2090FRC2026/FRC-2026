// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

  // TODO: Update this CAN ID to match your intake motor
  private static final int INTAKE_MOTOR_ID = 15;

  // Intake speeds (adjust as needed)
  private static final double INTAKE_SPEED = 0.5;
  private static final double OUTTAKE_SPEED = -0.3;
  private static final double SLOW_INTAKE_SPEED = 0.25;

  private final TalonFX motor;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public Intake() {
    motor = new TalonFX(INTAKE_MOTOR_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);

    motor.setPosition(0.0);
  }

  /**
   * Set the intake motor power.
   * @param power Motor power from -1.0 to 1.0
   */
  public void setPower(double power) {
    motor.setControl(dutyCycle.withOutput(MathUtil.clamp(power, -1.0, 1.0)));
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
