package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class PathTest extends Command{
    private SwerveSubsystem swerveSubsystem;
    
    public PathTest(SwerveSubsystem s){
        swerveSubsystem = s;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(0, 0, Rotation2d.kZero),
            new Pose2d(-.5, 0, Rotation2d.kZero),
            new Pose2d(-1 + 0.15, 0, Rotation2d.kZero)
        );

        PathConstraints constraints =  PathConstraints.unlimitedConstraints(12.0);

        PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, Rotation2d.kZero));

        path.preventFlipping = true;

        swerveSubsystem.resetPoseEstimate(new Pose2d(0, 0, Rotation2d.kZero));

        AutoBuilder.followPath(path).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
