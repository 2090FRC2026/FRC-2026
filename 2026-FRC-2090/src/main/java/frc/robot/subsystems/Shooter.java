// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
  private static final String kAppliedOutputEntry = "Shooter Applied Output";
  private static final String kVelocityEntry = "Shooter Velocity (RPM)";
  private static final String kTargetVelocityEntry = "Shooter Target RPM";
  private static final String kHoodOutputEntry = "Shooter Hood Output";

  // Motor CAN IDs
  private static final int MOTOR1_ID = 11;
  private static final int MOTOR2_ID = 13;
  private static final int MOTOR3_ID = 15; // Hood adjust motor

  // Velocity PIDF constants
  // The motor's built-in velocity PID handles the control
  private static final double kP = 0.22;//0.00004  // Proportional gain for velocity PID // 0.006
  private static final double kI = 0.0;   // Integral gain
  private static final double kD = 0.0;   // Derivative gain (set to 0 - can cause oscillation with velocity control) // 036
  private static final double kV = 0.1163;  // Feedforward velocity gain (volts per RPS) // 0.1166 - KEEP
  private static final double kS = 0.1;   // Feedforward static friction (volts) // 0.1

  // Target RPM presets
  public static final double SHOOTER_RPM = 3000.0;
  public static final double SHOOTER_RPM_LOW = 1500.0;

  // Hood manual power (tune later)
  public static final double HOOD_MANUAL_POWER = 0.05;

  private final TalonFX motor;
  private final TalonFX motor2;
  private final TalonFX motor3;
  private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private final DutyCycleOut hoodDutyCycle = new DutyCycleOut(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0.0);
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private double targetRPM = 0.0;

  public Shooter() {
    motor = new TalonFX(MOTOR1_ID);
    motor2 = new TalonFX(MOTOR2_ID);
    motor3 = new TalonFX(MOTOR3_ID);
    // Configure velocity PID gains
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    // Hood motor defaults (tune inversion + neutral mode later)
    motor3.setNeutralMode(NeutralModeValue.Brake);

    motor.setPosition(0.0);
    motor2.setPosition(0.0);
    motor3.setPosition(0.0);

    stop();

    // Configure velocity control to use Slot 0
    velocityControl.Slot = 0;

    stopHood();
  }

  /** Manual hood control (hold-to-move). Positive/negative direction may need flipping. */
  public void setHoodPower(double power) {
    motor3.setControl(hoodDutyCycle.withOutput(MathUtil.clamp(power, -1.0, 1.0)));
  }

  public void stopHood() {
    setHoodPower(0.0);
  }

  /**
   * Set both motors to a target RPM using velocity control.
   * Motor2 runs in the opposite direction.
   * @param rpm Target RPM (positive = intake direction)
   */
  public void setRPM(double rpm) {
    targetRPM = rpm;
    // Convert RPM to rotations per second (TalonFX velocity is in RPS)
    double targetRPS = rpm / 60.0;
    
    // Use the motor's built-in velocity PID
    motor.setControl(velocityControl.withVelocity(targetRPS));
    motor2.setControl(velocityControl.withVelocity(-targetRPS));
  }

  /**
   * Set the motors to a power level (open loop).
   * @param power Motor power from -1.0 to 1.0
   */
  public void setPower(double power) {
    targetRPM = 0.0;
    motor.setControl(dutyCycle.withOutput(MathUtil.clamp(power, -1.0, 1.0)));
    motor2.setControl(dutyCycle.withOutput(MathUtil.clamp(-power, -1.0, 1.0)));
  }

  /**
   * Stop both motors.
   */
  public void stop() {
    targetRPM = 0.0;
    motor.setControl(dutyCycle.withOutput(0.0));
    motor2.setControl(dutyCycle.withOutput(0.0));
  }

  /**
   * Get the current velocity of motor 1 in RPM.
   * @return Current RPM
   */
  public double getVelocityRPM() {
    // TalonFX reports velocity in rotations per second
    return motor.getVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Check if the shooter is at the target RPM (within tolerance).
   * @param toleranceRPM Acceptable error in RPM
   * @return true if at target
   */
  public boolean atTargetRPM(double toleranceRPM) {
    return Math.abs(getVelocityRPM() - targetRPM) < toleranceRPM;
  }

  /**
   * Check if the shooter is at target RPM
   * @return true if at target
   */
  public boolean atTargetRPM() {
    return atTargetRPM(10.0);
  }

  /**
   * Set the motor brake mode.
   * @param enabled true for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean enabled) {
    config.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // Log telemetry - the motor's built-in PID handles velocity control
    // No need to continuously update setControl - it maintains the target
    if (targetRPM != 0.0) {
      double currentRPM = getVelocityRPM();
      double errorRPM = targetRPM - currentRPM;
      
      SmartDashboard.putNumber("Shooter Error", errorRPM);
    }
    
    SmartDashboard.putNumber(kAppliedOutputEntry, motor.getDutyCycle().getValueAsDouble());
    SmartDashboard.putNumber(kVelocityEntry, getVelocityRPM());
    SmartDashboard.putNumber(kTargetVelocityEntry, targetRPM);
    SmartDashboard.putNumber(kHoodOutputEntry, motor3.getDutyCycle().getValueAsDouble());
  }

  // ===== Commands =====

  /**
   * Command to run the shooter at the default RPM.
   */
  public Command runAtRPM() {
    return run(() -> this.setRPM(SHOOTER_RPM))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the shooter at a specific RPM.
   * @param rpm Target RPM
   */
  public Command runAtRPM(double rpm) {
    return run(() -> this.setRPM(rpm))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the shooter at a continuously-updated RPM.
   * @param rpmSupplier Supplies the desired RPM each scheduler loop
   */
  public Command runAtRPM(DoubleSupplier rpmSupplier) {
    return run(() -> this.setRPM(rpmSupplier.getAsDouble()))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run the shooter at low speed.
   */
  public Command runAtLowRPM() {
    return run(() -> this.setRPM(SHOOTER_RPM_LOW))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to run motors at a fixed power (open loop).
   */
  public Command runMotors() {
    return run(() -> this.setPower(0.7))
        .finallyDo(() -> this.stop());
  }

  /**
   * Command to spin up to target RPM and wait until at speed.
   * @param rpm Target RPM
   */
  public Command spinUpAndWait(double rpm) {
    return runOnce(() -> this.setRPM(rpm))
        .andThen(run(() -> {}).until(() -> this.atTargetRPM()));
  }

  public Command hoodUpWhileHeld() {
    return run(() -> setHoodPower(HOOD_MANUAL_POWER))
        .finallyDo(this::stopHood);
  }

  public Command hoodDownWhileHeld() {
    return run(() -> setHoodPower(-HOOD_MANUAL_POWER))
        .finallyDo(this::stopHood);
  }
}
