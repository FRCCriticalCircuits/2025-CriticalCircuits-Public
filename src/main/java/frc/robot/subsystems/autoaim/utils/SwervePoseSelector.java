package frc.robot.subsystems.autoaim.utils;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.Math.AdvancedPose2D;

public class SwervePoseSelector {
    private static HashMap<Station, AdvancedPose2D> stationMapBlue = new HashMap<Station, AdvancedPose2D>(){{
        put(Station.A, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.B, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.C, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.D, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.E, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.F, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
    }};

    private static HashMap<Station, AdvancedPose2D> stationMapRed = new HashMap<Station, AdvancedPose2D>(){{
        put(Station.A, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.B, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.C, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.D, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.E, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
        put(Station.F, new AdvancedPose2D(0, 0, Rotation2d.fromDegrees(0)));
    }};

    private static double[] angleBoundaries = {-180.0, -150.0, -90.0, -30.0, 30.0, 90.0, 150.0, 180.0}; 
    private static Station[] station = {Station.D, Station.E, Station.F, Station.A, Station.B, Station.C, Station.D};

    private static Station estimateStation(double angle) {
        int left = 0;
        int right = angleBoundaries.length - 2;
    
        while (left <= right) {
            int middle = left + (right - left) / 2;
    
            if (angle >= angleBoundaries[middle] && angle < angleBoundaries[middle + 1]) {
                return station[middle];
            }
    
            if (angle < angleBoundaries[middle]) {
                right = middle - 1;
            } else {
                left = middle + 1;
            }
        }
        return Station.D;   // for theta = 180
    }

    public static AdvancedPose2D estimateStationAdvancedPose2D(Pose2d currentPose){
        Station station = estimateStation(currentPose.getRotation().getDegrees());
        if(DriveStationIO.isBlue()) return stationMapBlue.get(station);
        else return stationMapRed.get(station);
    }
    
    public enum Station{
        A,
        B,
        C,
        D,
        E,
        F
    }
}
