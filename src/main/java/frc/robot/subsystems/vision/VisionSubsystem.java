package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase{
    Notifier notifier;
    VisionIO cam_1, cam_2, cam_3;
    SwerveSubsystem swerveSubsystem;
    Optional<EstimatedRobotPose> result_1, result_2, result_3;

    public VisionSubsystem(){
        this.notifier = new Notifier(this::update);
        this.swerveSubsystem = SwerveSubsystem.getInstance(); 

        if(Robot.isReal()){
            // this.cam_1 = VisionOPI.getInstance("cam_1");
            // this.cam_2 = VisionOPI.getInstance("cam_2");
            // this.cam_3 = VisionLL.getInstance("limelight");
        }
    }

    public void start(){
        this.notifier.startPeriodic(0.01);
    }

    private void update(){
        result_3 = cam_3.getEstimatedGlobalPose(); 
        
        if(!result_3.isEmpty()){
            swerveSubsystem.updatePoseEstimator(
                result_3.get().estimatedPose.toPose2d(),
                result_3.get().timestampSeconds,
                cam_3.getEstimationStdDevs()
            );
        }
    }
}