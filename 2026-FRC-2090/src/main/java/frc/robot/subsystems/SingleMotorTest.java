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

public class SingleMotorTest extends SubsystemBase {
  private static final String kAppliedOutputEntry = "Single Motor Applied Output";
  private static final String kEncoderPositionEntry = "Single Motor Encoder Position";

  private final TalonFX motor;
  private final TalonFX motor2;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public SingleMotorTest() {
    motor = new TalonFX(11);
    motor2 = new TalonFX(13);

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    motor.setPosition(0.0);
    motor.setPosition(0);
  }

  public void setPower(double power) {
    motor.setControl(dutyCycle.withOutput(MathUtil.clamp(power, -1.0, 1.0)));
    motor2.setControl(dutyCycle.withOutput(MathUtil.clamp(power, -1.0, 1.0)));
  }

  public void stop() {
    motor.setControl(dutyCycle.withOutput(0.0));
    motor2.setControl(dutyCycle.withOutput(0.0));
  }

  public void setMotorBrake(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(kAppliedOutputEntry, motor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(kEncoderPositionEntry, motor.getPosition().getValueAsDouble());
  }

  // HOW TO SET POWER TO EACH INDIVIDUAL MOTOR DANIEL??? need to set motor2 to negative
  public Command runMotors() {
    return run(() -> this.setPower(0.05));
  }
}
