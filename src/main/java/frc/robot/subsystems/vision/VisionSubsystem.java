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
    private boolean enabled;

    public VisionSubsystem(){
        this.notifier = new Notifier(this::update);
        this.swerveSubsystem = SwerveSubsystem.getInstance(); 
        this.enabled = true;

        if(Robot.isReal()){
            this.limelight = VisionLL.getInstance("limelight");
        }

        this.notifier.setName("Vision");
    }

    public void start(){
        if(Robot.isReal()) this.notifier.startPeriodic(0.01);
    }

    public void enable() {
        this.enabled = true;
    }

    public void disable() {
        this.enabled = false;
    }

    private void update(){
        limelightResult = limelight.getEstimatedGlobalPose(); 

        if(swerveSubsystem.getGyroYawVelocity() > 12.5664) return;
  
        // Only update pose if limelight found a tag
        if(this.enabled && !limelightResult.isEmpty() && limelightResult.get().pose != null){
            swerveSubsystem.updatePoseEstimator(
                limelightResult.get().pose,
                limelightResult.get().timestampSeconds,
                limelightResult.get().stdDevs
            );
        }
    }
}