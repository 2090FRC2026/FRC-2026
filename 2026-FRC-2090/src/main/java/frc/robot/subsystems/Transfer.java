// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Transfer extends SubsystemBase {
  private static final String kAppliedOutputEntry = "Transfer Applied Output";
  private static final String kEncoderPositionEntry = "Transfer Encoder Position";

  private static final int TRANSFER_HIGH_MOTOR_ID = 20;
  private static final int TRANSFER_LOW_MOTOR_ID = 21;

  private static final double TRANSFER_SPEED = 0.5;
  private static final double SLOW_TRANSFER_SPEED = 0.25;

  private final TalonFX high_motor;
  private final SparkMax low_motor;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public Transfer() {
    high_motor = new TalonFX(TRANSFER_HIGH_MOTOR_ID);
    low_motor = new SparkMax(TRANSFER_LOW_MOTOR_ID, SparkLowLevel.MotorType.kBrushless); // neo motor

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    high_motor.getConfigurator().apply(config);

    high_motor.setPosition(0.0);
  }

  /**
   * Set the transfer motor power.
   * @param power Motor power from -1.0 to 1.0
   */
  public void setPower(double power) {
    double clampedPower = MathUtil.clamp(power, -1.0, 1.0);
    high_motor.setControl(dutyCycle.withOutput(clampedPower));
    low_motor.set(clampedPower);
  }

  /**
   * Stop the transfer motors.
   */
  public void stop() {
    high_motor.setControl(dutyCycle.withOutput(0.0));
    low_motor.set(0.0);
  }

  /**
   * Set the motor brake mode.
   * @param enabled true for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    high_motor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(kAppliedOutputEntry, high_motor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(kEncoderPositionEntry, high_motor.getPosition().getValueAsDouble());
  }

  // ===== Commands =====

  /**
   * Command to run the transfer at the default transfer speed.
   */
  public Command transferCommand() {
    return run(() -> this.setPower(-TRANSFER_SPEED))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the transfer at a slow speed.
   */
  public Command slowTransferCommand() {
    return run(() -> this.setPower(SLOW_TRANSFER_SPEED))
        .finallyDo(() -> this.stop());
  }

  /**
  * Command to run the transfer at a variable speed.
   * @param speed The speed to run at (-1.0 to 1.0)
   */
  public Command runAtSpeed(double speed) {
    return run(() -> this.setPower(speed))
        .finallyDo(() -> this.stop());
  }
}
