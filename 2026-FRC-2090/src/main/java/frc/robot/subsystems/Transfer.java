// Copied from intake, being modified for transfer

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.BeanSerializer;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class Transfer extends SubsystemBase {
  private static final String kAppliedOutputEntry = "Transfer Applied Output";
  private static final String kEncoderPositionEntry = "Transfer Encode ;;;'1r Position";

  // TODO: Update this CAN ID to match your TRANSFER motor
  private static final int TRANSFER_HIGH_MOTOR_ID = -1;
  private static final int TRANSFER_LOW_MOTOR_ID = -1; // this motor is a NEO hidden under rollers


  private static final double TRANSFER_SPEED = 0.5;
  private static final double SLOW_TRANSFER_SPEED = 0.25;
//   private static final double OUTTAKE_SPEED = -0.5;

  private final TalonFX high_motor;
  private final SparkMax low_motor;


  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public Transfer() {
    high_motor = new TalonFX(TRANSFER_HIGH_MOTOR_ID);
    low_motor = new SparkMax(TRANSFER_LOW_MOTOR_ID, SparkLowLevel.MotorType.kBrushless); // neo motor

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
    return run(() -> this.setPower(TRANSFER_SPEED))
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
   * Command to run the transfer in reverse (outtake).
   */

// Outtake func from intake, may not need
//   public Command outtakeCommand() {
//     return run(() -> this.setPower(/*ggykyuykugyugku*/))
//         .finallyDo(() -> this.stop());
//   }

  /**
   * Command to run the intake at a variable speed.
   * @param speed The speed to run at (-1.0 to 1.0)
   */
  public Command runAtSpeed(double speed) {
    return run(() -> this.setPower(speed))
        .finallyDo(() -> this.stop());
  }
}
