// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shooter subsystem using a Bang-Bang controller with feedforward.
 * 
 * Bang-Bang control is simple: if below target speed, apply full power.
 * If above target speed, apply no power (or coast).
 * 
 * Combined with feedforward, this provides fast spin-up and stable velocity.
 */
public class ShooterBangBang extends SubsystemBase {
  private static final String kVelocityEntry = "BangBang Shooter Velocity (RPM)";
  private static final String kTargetVelocityEntry = "BangBang Shooter Target RPM";
  private static final String kOutputEntry = "BangBang Shooter Output";
  private static final String kAtSetpointEntry = "BangBang At Setpoint";

  // Motor CAN IDs - UPDATE THESE FOR YOUR ROBOT
  private static final int MOTOR1_ID = 11;
  private static final int MOTOR2_ID = 13;

  // Feedforward constants (tune these for your shooter)
  // kS = static friction (volts needed to overcome friction)
  // kV = velocity gain (volts per RPM)
  private static final double kS = 0.25;
  private static final double kV = 0.002;  // Volts per RPM (adjust based on your motor's free speed)

  // Bang-Bang tolerance - how close to setpoint before we consider it "at speed"
  private static final double TOLERANCE_RPM = 50.0;

  // Target RPM presets
  public static final double DEFAULT_RPM = 3000.0;
  public static final double LOW_RPM = 1500.0;
  public static final double HIGH_RPM = 4500.0;

  private final TalonFX motor;
  private final TalonFX motor2;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Bang-Bang controller - built into WPILib
  private final BangBangController bangBang = new BangBangController(TOLERANCE_RPM);
  
  // Feedforward to help the bang-bang controller
  // This provides a baseline voltage to get close to target speed
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

  private double targetRPM = 0.0;
  private boolean enabled = false;

  public ShooterBangBang() {
    motor = new TalonFX(MOTOR1_ID);
    motor2 = new TalonFX(MOTOR2_ID);

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;  // Coast for flywheel
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    motor.setPosition(0.0);
    motor2.setPosition(0.0);
  }

  /**
   * Get the current velocity of the shooter in RPM.
   * @return Current RPM (average of both motors)
   */
  public double getVelocityRPM() {
    // TalonFX reports velocity in rotations per second, convert to RPM
    return motor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Get the current velocity of motor 2 in RPM.
   * @return Current RPM of motor 2
   */
  public double getMotor2VelocityRPM() {
    return Math.abs(motor2.getVelocity().getValueAsDouble() * 60.0);
  }

  /**
   * Set the target RPM for the shooter.
   * @param rpm Target RPM
   */
  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
    enabled = rpm > 0;
    bangBang.setSetpoint(rpm);
  }

  /**
   * Stop the shooter.
   */
  public void stop() {
    enabled = false;
    targetRPM = 0.0;
    motor.setControl(dutyCycle.withOutput(0.0));
    motor2.setControl(dutyCycle.withOutput(0.0));
  }

  /**
   * Check if the shooter is at the target RPM.
   * @return true if at setpoint
   */
  public boolean atSetpoint() {
    return bangBang.atSetpoint() && enabled;
  }

  /**
   * Check if the shooter is at a specific RPM within tolerance.
   * @param toleranceRPM Acceptable error in RPM
   * @return true if within tolerance
   */
  public boolean atTargetRPM(double toleranceRPM) {
    return Math.abs(getVelocityRPM() - targetRPM) < toleranceRPM && enabled;
  }

  /**
   * Set the motor brake mode.
   * @param enabled true for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean brakeEnabled) {
    config.MotorOutput.NeutralMode = brakeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    double currentRPM = getVelocityRPM();

    if (enabled && targetRPM > 0) {
      // Calculate bang-bang output (0 or 1)
      double bangBangOutput = bangBang.calculate(currentRPM);
      
      // Calculate feedforward voltage based on target RPM
      // This gets us close to the target, bang-bang fine-tunes it
      double ffVolts = feedforward.calculate(targetRPM);
      
      // Combine: feedforward provides baseline, bang-bang adds boost when needed
      // Normalize to duty cycle (-1 to 1) assuming 12V nominal
      double output = (ffVolts / 12.0) + (bangBangOutput * 0.2);  // Bang-bang adds 20% boost
      
      // Clamp output
      output = Math.min(output, 1.0);
      
      // Apply to motors (motor2 runs opposite direction)
      motor.setControl(dutyCycle.withOutput(output));
      motor2.setControl(dutyCycle.withOutput(-output));
      
      SmartDashboard.putNumber(kOutputEntry, output);
    } else {
      motor.setControl(dutyCycle.withOutput(0.0));
      motor2.setControl(dutyCycle.withOutput(0.0));
      SmartDashboard.putNumber(kOutputEntry, 0.0);
    }

    // Telemetry
    SmartDashboard.putNumber(kVelocityEntry, currentRPM);
    SmartDashboard.putNumber(kTargetVelocityEntry, targetRPM);
    SmartDashboard.putBoolean(kAtSetpointEntry, atSetpoint());
  }

  // ===== Commands =====

  /**
   * Command to run the shooter at the default RPM.
   */
  public Command runAtDefaultRPM() {
    return run(() -> this.setTargetRPM(DEFAULT_RPM))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the shooter at a specific RPM.
   * @param rpm Target RPM
   */
  public Command runAtRPM(double rpm) {
    return run(() -> this.setTargetRPM(rpm))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the shooter at low speed.
   */
  public Command runAtLowRPM() {
    return run(() -> this.setTargetRPM(LOW_RPM))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the shooter at high speed.
   */
  public Command runAtHighRPM() {
    return run(() -> this.setTargetRPM(HIGH_RPM))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to spin up to target RPM and end when at speed.
   * @param rpm Target RPM
   */
  public Command spinUpUntilReady(double rpm) {
    return runOnce(() -> this.setTargetRPM(rpm))
        .andThen(run(() -> {}).until(this::atSetpoint));
  }

  /**
   * Command to spin up and hold at target RPM, ending when at speed.
   * Use .andThen() to chain actions after spin-up.
   * @param rpm Target RPM
   */
  public Command spinUpAndHold(double rpm) {
    return run(() -> this.setTargetRPM(rpm))
        .until(this::atSetpoint);
  }
}
