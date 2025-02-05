package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionOPI implements VisionIO{
    private static HashMap<String, VisionOPI> instanceMap;

    /* Default Camera Position */
    private final Transform3d defaultCameraPos;

    /* Standard Deviations */
    private Matrix<N3, N1> curStdDevs, kSingleTagStdDevs, kMultiTagStdDevs;

    /* Camera Object */
    private PhotonCamera cam;
    
    /* PhotonPoseEstimator Object */
    private PhotonPoseEstimator photonEstimator;

    private VisionOPI(String name) {
        switch (name) {
            case "cam1":
                this.defaultCameraPos = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
                this.kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                this.kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
                break;
            case "cam2":
                this.defaultCameraPos = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
                this.kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                this.kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
                break;
            default:
                this.defaultCameraPos = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
                this.kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
                this.kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        }  

        this.cam = new PhotonCamera(name);
        this.curStdDevs = kSingleTagStdDevs;
        this.photonEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, defaultCameraPos);
        this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    public static VisionOPI getInstance(String name){
        if(instanceMap.get(name) == null) instanceMap.put(name, new VisionOPI(name));
        return instanceMap.get(name);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    @Override
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : cam.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    @Override
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}
