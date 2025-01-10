package frc.robot.utils.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.DataStrcutures.AutoAimSetting;
import frc.robot.utils.DataStrcutures.Spot;
import frc.robot.utils.DataStrcutures.Station;
import frc.robot.utils.DriveStationIO.DriveStationIO;

public class SwerveAimPose {
    /**U
     * Uses Binary Search to find the estimate station from robot heading
     * @param angle the robot's heading
     * @return the estimated {@link Station}
     */
    private static Station estimateStation(double angle) {
        int left = 0;
        int right = FieldConstants.AutoAim.BOUNDARIES.length - 2;
    
        while (left <= right) {
            int middle = left + (right - left) / 2;
    
            if (angle >= FieldConstants.AutoAim.BOUNDARIES[middle] && angle < FieldConstants.AutoAim.BOUNDARIES[middle + 1]) {
                return FieldConstants.AutoAim.STATIONS[middle];
            }
    
            if (angle < FieldConstants.AutoAim.BOUNDARIES[middle]) {
                right = middle - 1;
            } else {
                left = middle + 1;
            }
        }
        return Station.D;   // for theta = 180
    }

    /**
     * estimate the station driver heads
     * @param currentPose the current position of robot
     * @param spot The scoring Spot
     * @param manualTranslationX manual distence in meters
     * @return {@link AdvancePose2D} object for auto aiming
     */
    public static AdvancedPose2D estimateStationAdvancedPose2D(Pose2d currentPose, AutoAimSetting setting, Translation2d manualTranslation){
        Station station = estimateStation(currentPose.getRotation().getDegrees());
        AdvancedPose2D aimPose = DriveStationIO.isBlue() ? FieldConstants.AutoAim.STATION_BLUE.get(station) : FieldConstants.AutoAim.STATION_RED.get(station);

        // Apply Translations
        if(setting.spot == Spot.L){
            return aimPose.withVector(aimPose.getRotation(), new Translation2d(-FieldConstants.AutoAim.REEF_CENTER_TO_ROBOT, 0), aimPose.getRotation());
        }else if(setting.spot == Spot.R){
            return aimPose.withVector(aimPose.getRotation(), new Translation2d(FieldConstants.AutoAim.REEF_CENTER_TO_ROBOT, 0), aimPose.getRotation());
        }else{
            return aimPose.withVector(aimPose.getRotation(), new Translation2d(manualTranslation.getX(), manualTranslation.getY()), aimPose.getRotation());
        }
    }
}
