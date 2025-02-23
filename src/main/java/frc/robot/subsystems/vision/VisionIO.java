package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    public static class VisionResult{
        public Pose2d pose;
        public double timestampSeconds;
        public Matrix<N3, N1> stdDevs;
    }

    public Optional<VisionResult> getEstimatedGlobalPose();
}
