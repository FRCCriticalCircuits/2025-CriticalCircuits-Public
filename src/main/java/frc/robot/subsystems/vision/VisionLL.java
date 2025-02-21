package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.LimelightHelpers;

public class VisionLL implements VisionIO{
    private static HashMap<String, VisionLL> instanceMap = new HashMap<String, VisionLL>();

    private String camName;

    public static VisionLL getInstance(String name){
        if(instanceMap.get(name) == null) instanceMap.put(name, new VisionLL(name));
        return instanceMap.get(name);
    }

    public VisionLL(String name) {
        this.camName = name;
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(camName);
        return Optional.of(
            new EstimatedRobotPose(
                new Pose3d(
                    new Translation3d(mt2.pose.getTranslation()),
                    new Rotation3d(mt2.pose.getRotation())
                ),
                mt2.timestampSeconds,
                null,
                null
            )
        );
    }

    @Override
    public Matrix<N3, N1> getEstimationStdDevs() {
        return VecBuilder.fill(0.2, 0.2, 0.1);
    }
}
