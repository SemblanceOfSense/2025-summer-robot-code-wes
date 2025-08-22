package frc.robot.commands.swervedrive.drivebase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToPoint extends Command {
    private SwerveSubsystem swerveSubsystem;
    private Pose2d desiredPose;

    public DriveToPoint(SwerveSubsystem swerveSubsystem, Pose2d desiredPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.desiredPose = desiredPose;
    }

    @Override
    public void execute() {
        swerveSubsystem.DriveToPoint(desiredPose);
    }
}
