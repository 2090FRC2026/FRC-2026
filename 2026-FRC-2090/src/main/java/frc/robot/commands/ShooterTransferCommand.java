package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class ShooterTransferCommand extends ParallelCommandGroup {
    public ShooterTransferCommand(Shooter shooter, Transfer transfer) {
        addCommands(
            new RunCommand(() -> shooter.runAtRPM(2000), shooter),
            new RunCommand(() -> transfer.transferCommand(), transfer)
        );
        // Optionally set a timeout:
        // withTimeout(2);
    }
}
