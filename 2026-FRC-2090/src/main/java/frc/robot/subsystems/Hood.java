package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

    private final TalonFX hoodMotor;

    private static final int HOOD_MOTOR_ID = 31;

    // Hood speed (adjust to taste)
    private static final double HOOD_UP_SPEED = 0.1;
    private static final double HOOD_DOWN_SPEED = -0.1;

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public Hood() {
        hoodMotor = new TalonFX(HOOD_MOTOR_ID);

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hoodMotor.getConfigurator().apply(config);
        hoodMotor.setPosition(0.0);

        stop();
    }

    /**
     * Set the hood motor power.
     * @param power Motor power from -1.0 to 1.0
     */
    public void setPower(double power) {
        hoodMotor.setControl(dutyCycle.withOutput(MathUtil.clamp(power, -1.0, 1.0)));
    }

    /** Stop the hood motor. */
    public void stop() {
        hoodMotor.setControl(dutyCycle.withOutput(0.0));
    }

    /**
     * Get the hood motor position in rotations.
     * @return Position in motor rotations
     */
    public double getPosition() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood Position", getPosition());
        SmartDashboard.putNumber("Hood Output", hoodMotor.getDutyCycle().getValueAsDouble());
    }

    // ===== Commands =====

    /** Command to move the hood up while held. */
    public Command hoodUpCommand() {
        return run(() -> setPower(HOOD_UP_SPEED))
            .finallyDo(() -> stop());
    }

    /** Command to move the hood down while held. */
    public Command hoodDownCommand() {
        return run(() -> setPower(HOOD_DOWN_SPEED))
            .finallyDo(() -> stop());
    }

    /**
     * Command to control hood speed from a variable input (e.g. trigger axis).
     * Right trigger (0 to 1) moves hood up, left trigger (0 to -1) moves hood down.
     * @param speedSupplier Supplier of speed value from -1.0 to 1.0
     */
    public Command variableHoodCommand(DoubleSupplier speedSupplier) {
        return run(() -> setPower(speedSupplier.getAsDouble()))
            .finallyDo(() -> stop());
    }
}
