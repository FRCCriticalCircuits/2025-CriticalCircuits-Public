package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionIO.VisionResult;

public class VisionSubsystem extends SubsystemBase{
    Notifier notifier;
    VisionIO limelight;
    SwerveSubsystem swerveSubsystem;
    Optional<VisionResult> limelightResult;

    public VisionSubsystem(){
        this.notifier = new Notifier(this::update);
        this.swerveSubsystem = SwerveSubsystem.getInstance(); 

        if(Robot.isReal()){
            this.limelight = VisionLL.getInstance("limelight");
        }

        this.notifier.setName("Vision");
    }

    public void start(){
        if(Robot.isReal()) this.notifier.startPeriodic(0.01);
    }

    private void update(){
        limelightResult = limelight.getEstimatedGlobalPose(); 
  
        if(!limelightResult.isEmpty() && limelightResult.get().pose != null){
            swerveSubsystem.updatePoseEstimator(
                limelightResult.get().pose,
                limelightResult.get().timestampSeconds,
                limelightResult.get().stdDevs
            );
        }
    }
}