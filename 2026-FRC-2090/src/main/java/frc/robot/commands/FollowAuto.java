// package frc.robot.commands;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SwerveDrive;


// public class FollowAuto extends Command {
//   private final SwerveDrive swerveDrive;
//   private final Command autoCommand;

//   public FollowPathAuto(SwerveDrive swerveDrive) {
//     this.swerveDrive = swerveDrive;
//     autoCommand = new PathPlannerAuto("center_preload_blue_auto");
//     addRequirements(swerveDrive);
//   }

//   @Override
//   public void initialize() {
//     swerveCommand.initialize();
//   }

//   @Override
//   public void execute() {
//     swerveCommand.execute();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     swerveCommand.end(interrupted);
//   }

//   @Override
//   public boolean isFinished() {
//     return swerveCommand.isFinished();
//   }
// }
