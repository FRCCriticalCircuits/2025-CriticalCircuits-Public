package frc.robot.subsystems.autoaim;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.Math.AdvancedPose2D;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;
import frc.robot.utils.structures.DataStrcutures.Station;

public class AutoAimManager{
    public static AutoAimManager instance;

    public AutoAimSetting setting = new AutoAimSetting(Spot.MID, Level.L1, Mode.PLACE);

    private double[] BOUNDARIES = {-180.0, -150.0, -90.0, -30.0, 30.0, 90.0, 150.0, 180.0}; 
    private Station[] STATIONS = {Station.D, Station.E, Station.F, Station.A, Station.B, Station.C, Station.D};
    
    private double currentAngle = 0;
    private Field2d field2d = new Field2d();
    
    private PathConstraints constraints = new PathConstraints(
        1,
        0.3,
        Math.PI / 4,
        Math.PI / 2
    );

    public AutoAimManager(){}
    
    public static AutoAimManager getInstance(){
        if(instance == null) instance = new AutoAimManager();
        return instance;
    }

    /**
     * Uses Binary Search to find the estimate station from robot heading
     * @param angle the robot's heading
     * @return the estimated {@link Station}
     */
    public Station estimateStation(double angle) {
        for (int i = 0; i < BOUNDARIES.length - 1; i++) {
            if (angle >= BOUNDARIES[i] && angle < BOUNDARIES[i + 1]) {
                return STATIONS[i];
            }
        }
        return Station.D; 
    }

    /**
     * estimate the station driver heads
     * @param currentPose the current position of robot
     * @param setting Autoaim Setting Includes {@link Spot}, {@link Level}, {@link Mode}
     * @param manualTranslationX manual distence in meters
     * @return {@link AdvancePose2D} object for auto aiming
     */
    public AdvancedPose2D estimateStationAdvancedPose2D(AutoAimSetting setting, Translation2d manualTranslation){
        Station station = estimateStation(currentAngle);
        AdvancedPose2D aimPose = DriveStationIO.isBlue() ? FieldConstants.AutoAim.STATION_BLUE.get(station) : FieldConstants.AutoAim.STATION_RED.get(station);

        // Apply Translations
        if(setting.spot == Spot.L){
            return aimPose.withVector(aimPose.getRotation(), new Translation2d(-FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET_X, 0), aimPose.getRotation());
        }else if(setting.spot == Spot.R){
            return aimPose.withVector(aimPose.getRotation(), new Translation2d(FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET_X, 0), aimPose.getRotation());
        }else{
            return aimPose.withVector(aimPose.getRotation(), new Translation2d(manualTranslation.getX(), manualTranslation.getY()), aimPose.getRotation());
        }
    }

    public void updateAngle(double angleDegree){
        this.currentAngle = angleDegree;
    }

    public Command runSwerveAutoAim(Translation2d manualTranslation){
        Pose2d targetPose = estimateStationAdvancedPose2D(setting, manualTranslation);

        field2d.setRobotPose(targetPose);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity in meters/sec
        );
    }
}
