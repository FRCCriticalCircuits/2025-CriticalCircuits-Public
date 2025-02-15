package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.Math.AdvancedPose2D;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;
import frc.robot.utils.structures.DataStrcutures.Station;
import frc.robot.utils.web.WebServer;

public class AutoAimManager{
    private static AutoAimManager instance;
    
    private WebServer server = WebServer.getInstance();
    private Notifier notifier = new Notifier(this::updateValues);

    private Command command;

    private AutoAimSetting setting;
    private Supplier<Double> LTSupplier, RTSupplier;

    private Pose2d targetPose = new Pose2d();
    private double[] BOUNDARIES = {-180.0, -150.0, -90.0, -30.0, 30.0, 90.0, 150.0, 180.0}; 
    private Station[] STATIONS = {Station.D, Station.E, Station.F, Station.A, Station.B, Station.C, Station.D};
    private Field2d field2d = new Field2d();
    
    private PathConstraints constraints = new PathConstraints(
        1.3,
        2,
        Math.PI / 4,
        Math.PI / 2
    );

    private AutoAimManager(Supplier<Double> LTSupplier, Supplier<Double> RTSupplier){
        this.LTSupplier = LTSupplier;
        this.RTSupplier = RTSupplier;

        notifier.startPeriodic(0.05);
    }
    
    public synchronized static AutoAimManager getInstance(Supplier<Double> LTSupplier, Supplier<Double> RTSupplier){
        if(instance == null) instance = new AutoAimManager(LTSupplier, RTSupplier);
        return instance;
    }

    /**
     * Find the cloest coral station based on current Pos
     * @param currentPos current {@link Translation2d} for the robot
     * @return the pose for target coral station
     */
    private AdvancedPose2D cloestCoralStation(Translation2d currentPos){
        if(DriveStationIO.getAlliance() == Alliance.Blue){
            if (
                currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_A.getTranslation()) > 
                currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_B.getTranslation())
            ) return FieldConstants.AutoAim.CORAL_STATION_B;
            else return FieldConstants.AutoAim.CORAL_STATION_A;
        }else{
            if (
                currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_C.getTranslation()) > 
                currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_D.getTranslation())
            ) return FieldConstants.AutoAim.CORAL_STATION_D;
            else return FieldConstants.AutoAim.CORAL_STATION_C;
        }
    }

    /**
     * Uses Binary Search to find the estimate station from robot heading
     * @param angle the robot's heading
     * @return the estimated {@link Station}
     */
    private Station estimateStation(double angle) {
        int left = 0;
        int right = BOUNDARIES.length - 2;
    
        while (left <= right) {
            int middle = left + (right - left) / 2;
    
            if (angle >= BOUNDARIES[middle] && angle < BOUNDARIES[middle + 1]) {
                return STATIONS[middle];
            }
    
            if (angle < BOUNDARIES[middle]) {
                right = middle - 1;
            } else {
                left = middle + 1;
            }
        }

        return Station.D;   // for theta = 180
    }

    /**
     * estimate the station based on current heading
     * @param setting Autoaim Setting Includes {@link Spot}, {@link Level}, {@link Mode}
     * @param manualTranslation manual translation distence from the origin aim pos
     * @return {@link AdvancedPose2D} for pathplanning
     */
    private AdvancedPose2D estimateAimPos(AutoAimSetting setting, Translation2d manualTranslation){
        if(setting.getMode() == Mode.CORAL_INTAKE){
            return cloestCoralStation(SwerveSubsystem.getInstance().getPoseEstimate().getTranslation());
        }else{
            Station station = estimateStation(SwerveSubsystem.getInstance().getPoseEstimate().getRotation().getDegrees());
            AdvancedPose2D aimPose = DriveStationIO.isBlue() ? FieldConstants.AutoAim.STATION_BLUE.get(station) : FieldConstants.AutoAim.STATION_RED.get(station);

            if(setting.getSpot() == Spot.L){
                return aimPose.withRobotRelativeTransformation(new Translation2d(-FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET_X, 0));
            }else if(setting.getSpot() == Spot.R){
                return aimPose.withRobotRelativeTransformation(new Translation2d(FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET_X, 0));
            }else{
                return aimPose.withRobotRelativeTransformation(manualTranslation);
            }
        }
    }

    /**
     * a perodic funtion (0.05s) updates values from WebSocket, save/send Estimate Target Pose
     */
    private void updateValues() {
        setting = server.getAutoAimSettings();

        targetPose = estimateAimPos(
            setting,
            new Translation2d
            (
                FieldConstants.AutoAim.MANUAL_TRANSLATION_RANGE * (LTSupplier.get() - RTSupplier.get()),
                0
            )
        );
        
        field2d.setRobotPose(targetPose);

        SmartDashboard.putData("Swerve AutoAim", field2d);
    }

    /**
     * get the PathFinding Command based on current settings
     * @return the {@link Command} to execute
     */
    public synchronized Command getCommand(){
        updateValues();

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        command = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity in meters/sec
        );

        return command;
    }

    public synchronized boolean isFinished(){
        return command.isFinished();
    }

    public synchronized void cancle(){
        command.cancel();
    }
}
