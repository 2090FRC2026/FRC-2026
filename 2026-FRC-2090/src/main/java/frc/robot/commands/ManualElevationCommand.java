package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevationCommand extends Command {
  private final CommandXboxController controller;

  private double targetPosition = 0;

  private final ElevatorSubsystem elevator;

  public ManualElevationCommand(ElevatorSubsystem elevator, CommandXboxController controller) {
    this.controller = controller;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
  }

  @Override // every 20ms
  public void execute() {
    double input = -controller.getLeftY();

    // Only update the target position if input is outside the deadband
    if (Math.abs(input) > Constants.ElevatorConstants.JOYSTICK_DEADBAND) {
      targetPosition += input * 0.5; // Scale input to avoid excessive movement
    }

    // Clamp the target position to within the valid elevator range
    targetPosition = Math.max(Math.min(targetPosition, Constants.ElevatorConstants.MAX_HEIGHT_INCHES),
        Constants.ElevatorConstants.MIN_HEIGHT_INCHES);

    // Continuously set the elevator to the last stored target position
    elevator.setPosition(targetPosition);
  }
}

  