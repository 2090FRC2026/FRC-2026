// package frc.robot.commands.swervedrive.auto;

// import com.pathplanner.lib.commands.PathPlannerAuto;

// import edu.wpi.first.wpilibj2.command.Command;


// // YAGSL drive system instea of swerve drive
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class FollowAuto extends Command {
//   private final SwerveSubsystem swerveSubsystem;
//   private final Command autoCommand;

//   public FollowAuto(SwerveSubsystem swerveDrive) {
//     this.swerveSubsystem = swerveDrive;
//     autoCommand = new PathPlannerAuto("center_preload_blue_auto");
//     addRequirements(swerveSubsystem);
//   }

//   @Override
//   public void initialize() {
//     autoCommand.initialize();
//   }

//   @Override
//   public void execute() {
//     autoCommand.execute();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     autoCommand.end(interrupted);
//   }

//   @Override
//   public boolean isFinished() {
//     return autoCommand.isFinished();
//   }
// }
