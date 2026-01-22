package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ArmConstants.ArmStates;
import frc.robot.Constants.ScoringConstants.ScoringStates;
import frc.robot.Constants.WristConstants.WristStates;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.AlgaeIntake;

public class ScoringMacro extends Command {
    // private final ArmSubsystem arm;
    // private final ElevatorSubsystem elevator;
    // private final WristSubsystem wrist;

    private final ScoringSubsystem scoring;

    private final ElevatorSubsystem elevator;

    private ScoringStates state;
    
    public ScoringMacro(ScoringSubsystem scoring, ElevatorSubsystem elevator, ScoringStates state) {
        this.scoring = scoring;
        this.state = state;
        this.elevator = elevator;
        addRequirements(scoring, elevator);
    }

    public void initialize() {

    }

    @Override
    public void execute() {
        switch (state) {
            case Stow:
                scoring.setArmPivotState(state);
                // scoring.setWristState(state);
                elevator.setElevatorState(state);
                break;
        
            case Intake:
                // scoring.reachArmPivotTarget(ArmConstants.intake_angle);
                scoring.setArmPivotState(state);
                elevator.setElevatorState(state);
                break;
            case L1:
                elevator.setElevatorState(state);
                break;
            case L2:
                elevator.setElevatorState(state);
                break;
            case L3:
                elevator.setElevatorState(state);
                break;
            
        }
    }

    @Override
    public void end(boolean isFinished) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
