package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {
    public static class VisionResult{
        public Pose2d pose;
        public double timestampSeconds;
        public Matrix<N3, N1> stdDevs;

        public VisionResult() {
            this.pose = new Pose2d();
            this.timestampSeconds = 0;
            this.stdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
        }
    }

    public Optional<VisionResult> getEstimatedGlobalPose();
}
