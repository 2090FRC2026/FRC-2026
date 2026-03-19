package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class ShooterTransferCommand extends ParallelCommandGroup {
    public ShooterTransferCommand(Shooter shooter, Transfer transfer) {
        addCommands(
            shooter.runAtRPM(2250),
            transfer.transferCommand()
        );
        // Optionally set a timeout:
        // withTimeout(2);
    }
}
