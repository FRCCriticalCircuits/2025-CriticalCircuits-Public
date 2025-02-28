package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Optional;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;

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

    
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link VisionResult} with an estimated pose, estimate timestamp, and stdDev
     *     for estimation.
     */
    @Override
    public Optional<VisionResult> getEstimatedGlobalPose() {
        PoseEstimate visionEst = LimelightHelpers.getBotPoseEstimate_wpiBlue(camName);

        if(LimelightHelpers.validPoseEstimate(visionEst)){
            VisionResult result = new VisionResult(); 
            result.pose = visionEst.pose;
            result.timestampSeconds = visionEst.timestampSeconds;
            result.stdDevs = LimelightHelpers.getMT1StdDevs(camName);
            
            return Optional.of(result);
        }else{
            return Optional.empty();
        }
    }
}
