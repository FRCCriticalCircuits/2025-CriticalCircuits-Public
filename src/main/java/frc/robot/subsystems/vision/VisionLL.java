package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

public class VisionLL implements VisionIO{
    private static HashMap<String, VisionLL> instanceMap = new HashMap<String, VisionLL>();

    /* Standard Deviations */
    private Matrix<N3, N1> curStdDevs, kSingleTagStdDevs, kMultiTagStdDevs;
    private String camName;

    public static VisionLL getInstance(String name){
        if(instanceMap.get(name) == null) instanceMap.put(name, new VisionLL(name));
        return instanceMap.get(name);
    }

    public VisionLL(String name) {
        switch(name){
            case "limelight":
                kSingleTagStdDevs = VecBuilder.fill(0.25, 0.25, 0.5);
                kMultiTagStdDevs = VecBuilder.fill(0.15, 0.15, 0.2);
                break;
            default:
                throw new IllegalArgumentException("Invalid Camera Name");
        }
        this.camName = name;
    }

    
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link VisionResult} with an estimated pose, estimate timestamp, and stdDev
     *     for estimation.
     */
    @Override
    public Optional<VisionResult> getEstimatedGlobalPose() {
        Optional<PoseEstimate> visionEst = Optional.of(LimelightHelpers.getBotPoseEstimate_wpiBlue(camName));
        VisionResult result = new VisionResult();
        if(!visionEst.isEmpty()){
            result.pose = visionEst.get().pose;
            result.timestampSeconds = visionEst.get().timestampSeconds;
            updateEstimationStdDevs(visionEst);
            result.stdDevs = curStdDevs;
            return Optional.of(result);
        }else{
            return Optional.empty();
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedData The estimated data to guess standard deviations for
     */
    private void updateEstimationStdDevs(Optional<PoseEstimate> estimatedData) {
        if (estimatedData.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = estimatedData.get().tagCount;
            double avgDist = estimatedData.get().avgTagDist;

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Set std devs if too far (3.5 meters)
                if (numTags == 1 && avgDist > 3.5)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 15));
                curStdDevs = estStdDevs;
            }
        }
    }
}
