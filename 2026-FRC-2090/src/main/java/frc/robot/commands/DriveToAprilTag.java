package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToAprilTag extends Command {

    private final SwerveSubsystem driveTrain;

    private final VisionSubsystem limelightSubsystem;

    private Command driveToPoseCommand;

    public DriveToAprilTag(SwerveSubsystem driveTrain, VisionSubsystem limelightSubsystem) {
        this.driveTrain = driveTrain;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(driveTrain, limelightSubsystem);
    }

    @Override
    public void initialize() {
        var visionEstimate = limelightSubsystem.GetVisionEstimate();
        if (visionEstimate != null && visionEstimate.pose != null) {
            Pose2d targetPose = visionEstimate.pose;

            // driveToPoseCommand = driveTrain.driveToPoseWithVision(targetPose, limelightSubsystem);
            driveToPoseCommand.initialize();
        } else {
            System.out.println("no tag available for detection");
            driveToPoseCommand = null;
        }
    }

    @Override
    public void execute() {
        if (driveToPoseCommand != null) {
            driveToPoseCommand.execute();
        }   

    }

    @Override
    public void end(boolean isInterrupted) {
        if (driveToPoseCommand != null) {
            // driveToPoseCommand.end(interrupted);
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}