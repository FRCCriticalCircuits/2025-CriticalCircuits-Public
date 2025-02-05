package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose();
    public Matrix<N3, N1> getEstimationStdDevs();
}
