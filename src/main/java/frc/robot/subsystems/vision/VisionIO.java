package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose();
}
