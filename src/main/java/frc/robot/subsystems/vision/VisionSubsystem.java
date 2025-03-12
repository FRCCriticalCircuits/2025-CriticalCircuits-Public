package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
// import frc.robot.subsystems.vision.VisionIO.VisionResult;
import frc.robot.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase{
    Notifier notifier;
    SwerveSubsystem swerveSubsystem;
    // Optional<VisionResult> limelightResult;
    private boolean enabled;

    public VisionSubsystem(SwerveSubsystem s){
        this.notifier = new Notifier(this::update);
        this.swerveSubsystem = s; 
        this.enabled = true;

        // if(Robot.isReal()){
        //     this.limelight = VisionLL.getInstance("limelight");
        // }

        this.notifier.setName("Vision");
    }

    public void start(){
        if(Robot.isReal()) this.notifier.startPeriodic(0.02);
    }

    public void enable() {
        this.enabled = true;
    }

    public void disable() {
        this.enabled = false;
    }

    private void update(){
      // experimental mt2 LL
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      
      boolean rejectUpdate = false;

      // Do not run if disabled
      if (!this.enabled) rejectUpdate = true;

      if (Math.abs(swerveSubsystem.getGyroYawVelocity()) > 2*Math.PI) {
        rejectUpdate = true;
      }

      if (mt2.tagCount == 0) rejectUpdate = true;

      if (!rejectUpdate) {
        swerveSubsystem.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        swerveSubsystem.getPoseEstimator().addVisionMeasurement(mt2.pose, mt2.timestampSeconds);

      }
      // limelightResult = limelight.getEstimatedGlobalPose(); 

        // if(swerveSubsystem.getGyroYawVelocity() > 12.5664) return;
  
        // // Only update pose if limelight found a tag
        // if(this.enabled && !limelightResult.isEmpty() && limelightResult.get().pose != null){
        //     swerveSubsystem.updatePoseEstimator(
        //         limelightResult.get().pose,
        //         limelightResult.get().timestampSeconds,
        //         limelightResult.get().stdDevs
        //     );
        // }
    }
}