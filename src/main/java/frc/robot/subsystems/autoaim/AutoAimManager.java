package frc.robot.subsystems.autoaim;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.DataStrcutures.AutoAimSetting;
import frc.robot.utils.Math.SwerveAimPose;

public class AutoAimManager {
    public static AutoAimSetting setting;
    
    private static SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

    public static void configure(AutoAimSetting setting){
        AutoAimManager.setting = setting;
    }

    public static Command runSwerveAutoAim(Translation2d manualTranslation){
        Pose2d targetPose = SwerveAimPose.estimateStationAdvancedPose2D(swerveSubsystem.getPoseEstimate(), setting, manualTranslation);
        
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
            1.5,
            0.5,
            Units.degreesToRadians(10),
            Units.degreesToRadians(5)
        );

        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0
        );
    }
}
