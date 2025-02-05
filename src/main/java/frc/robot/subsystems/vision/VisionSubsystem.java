package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class VisionSubsystem extends SubsystemBase{
    Notifier notifier;
    VisionIO cam_1, cam_2;
    SwerveSubsystem swerveSubsystem;

    public VisionSubsystem(){
        this.notifier = new Notifier(this::update);
        this.swerveSubsystem = SwerveSubsystem.getInstance(); 

        if(Robot.isReal()){
            this.cam_1 = VisionOPI.getInstance("cam_1");
            this.cam_2 = VisionOPI.getInstance("cam_2");
        }

        this.notifier.startPeriodic(0.01);
    }

    private void update(){
        swerveSubsystem.updatePoseEstimator(
            null,
            0, 
            null
        );
        swerveSubsystem.updatePoseEstimator(
            null,
            0, 
            null
        );
        swerveSubsystem.updatePoseEstimator(
            null,
            0, 
            null
        );
    }
}
