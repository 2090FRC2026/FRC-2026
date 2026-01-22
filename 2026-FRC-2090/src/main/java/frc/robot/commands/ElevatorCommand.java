package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants.RobotStates;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private double elevateTarget;

    private ElevatorSubsystem elevator;

    private static final double ELEVATE_COMMAND_DEADBAND = 0.5; // inches

    public ElevatorCommand(ElevatorSubsystem elevator, double elevateTarget) {
        this.elevator = elevator;
        
        elevateTarget = Math.max(Math.min(elevateTarget, Constants.ElevatorConstants.MAX_HEIGHT_INCHES),
          Constants.ElevatorConstants.MIN_HEIGHT_INCHES);
        this.elevateTarget = elevateTarget;
        addRequirements(elevator);
      // It's fine to ignore the joystick because the profile should not take very
      // long
    }

    @Override // every 20ms
    public void execute() {
      SmartDashboard.putNumber("Elevator Target", elevateTarget);
      elevator.setPosition(elevateTarget);
    }

    @Override
    public void end(boolean isInterrupted) {
      elevator.setPosition(elevator.getElevatorPosition());
    }

    @Override
    public boolean isFinished() {
      return Math.abs(elevator.getElevatorPosition() - elevateTarget) < ELEVATE_COMMAND_DEADBAND;
    }
  }
