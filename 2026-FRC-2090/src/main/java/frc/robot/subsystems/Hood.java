package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

    private final TalonFX hoodMotor;

    private static final int HOOD_MOTOR_ID = 31;

    // Hood speed (adjust to taste)
    private static final double HOOD_UP_SPEED = 0.001;
    private static final double HOOD_DOWN_SPEED = -0.001;

    // Hood positions (adjust as needed)
    private static final double HOOD_POS_DOWN = 0.0;
    private static final double HOOD_POS_MID = 1.5;
    private static final double HOOD_POS_UP = 3.0;

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
    private final PositionVoltage hoodPosition = new PositionVoltage(0.0);
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public Hood() {
        hoodMotor = new TalonFX(HOOD_MOTOR_ID);

        config.Slot0.kP = 2.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 10;
        config.Slot0.kS = 10;  

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

    public enum HoodPosition {
        UP, MIDDLE, DOWN
    }

    private HoodPosition currentHoodPosition = HoodPosition.UP;
    
    /** Toggle hood between up, mid, and down positions. */
    public void toggleHood() {
        switch (currentHoodPosition) {
            case UP:
                hoodMotor.setControl(hoodPosition.withPosition(HOOD_POS_MID));
                currentHoodPosition = HoodPosition.MIDDLE;
                break;
            case MIDDLE:
                hoodMotor.setControl(hoodPosition.withPosition(HOOD_POS_DOWN));
                currentHoodPosition = HoodPosition.DOWN;
                break;
            case DOWN:
                hoodMotor.setControl(hoodPosition.withPosition(HOOD_POS_UP));
                currentHoodPosition = HoodPosition.UP;
                break;
        }
    }

    public void setHoodUp()     { hoodMotor.setControl(hoodPosition.withPosition(HOOD_POS_UP));   currentHoodPosition = HoodPosition.UP; }
    public void setHoodMiddle() { hoodMotor.setControl(hoodPosition.withPosition(HOOD_POS_MID));  currentHoodPosition = HoodPosition.MIDDLE; }
    public void setHoodDown()   { hoodMotor.setControl(hoodPosition.withPosition(HOOD_POS_DOWN)); currentHoodPosition = HoodPosition.DOWN; }

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

    public Command toggleHoodCommand() {
        return runOnce(this::toggleHood);
    }

    /**
     * Command to control hood speed from a variable input (e.g. trigger axis).
     * Right trigger (0 to 1) moves hood up, left trigger (0 to -1) moves hood down.
     * @param speedSupplier Supplier of speed value from -1.0 to 1.0
     */
    public Command variableHoodCommand(DoubleSupplier speedSupplier) {
        return run(() -> setPower(speedSupplier.getAsDouble() * 0.1))
            .finallyDo(() -> stop());
    }
}
