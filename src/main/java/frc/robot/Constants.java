package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.DataStrcutures.Station;
import frc.robot.utils.Math.AdvancedPose2D;

public class Constants {
    public class DeviceID {
        public static final int GAMEPAD_DRIVER = 0;
        
        public class DriveBase{
            public static final int FRONT_LEFT_DRIVE_ID = 1;
            public static final int FRONT_LEFT_TURN_ID = 2;
            public static final int FRONT_LEFT_CANCODER_ID = 3;
        
            public static final int FRONT_RIGHT_DRIVE_ID = 4;
            public static final int FRONT_RIGHT_TURN_ID = 5;
            public static final int FRONT_RIGHT_CANCODER_ID = 6;
        
            public static final int REAR_LEFT_DRIVE_ID = 7;
            public static final int REAR_LEFT_TURN_ID = 8;
            public static final int REAR_LEFT_CANCODER_ID = 9;
        
            public static final int REAR_RIGHT_DRIVE_ID = 10;
            public static final int REAR_RIGHT_TURN_ID = 11;
            public static final int REAR_RIGHT_CANCODER_ID = 12;

            public static final int GYRO_CAN_ID = 20;
        }
    }

    public class TunedConstants{
        public class DriveBase{
            public static double DRIVE_PID_P = 0.1;
            public static double DRIVE_PID_I = 0;

            public static double DRIVE_FEED_FORWARD_KS = 0.015;
            public static double DRIVE_FEED_FORWARD_KV = 0.17; // 0.17V -> RPM
            public static double DRIVE_FEED_FORWARD_KA = 0;

            public static double TURN_PID_P = 0.3;
            public static double TURN_PID_I = 0;
            public static double TURN_PID_D = 0;
        }
    }

    public class PhysicalConstants {
        public static double NOMINAL_VOLTAGE = 12;
        
        public class DriveBase {
            public class LENGTHS{
                public static double TRACK_WIDTH_METERS = 0.55245;
                public static double TRACK_RADIUS_METERS = Math.sqrt((TRACK_WIDTH_METERS * TRACK_WIDTH_METERS) + (TRACK_WIDTH_METERS * TRACK_WIDTH_METERS));

                public static double DRIVE_WHEEL_DIAMETER_INCHES = 4;
                public static double DRIVE_WHEEL_DIAMETER_METERS = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER_INCHES);
                public static double DRIVE_WHEEL_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER_METERS * Math.PI;
            }

            public class GearRatios{
                public static double DRIVE_GEAR_RATIO = 6.12;
                public static double TURN_GEAR_RATIO = 150.0 / 7.0;
            }      

            public class CurrentLimits{
                public static int DRIVE_CURRENT_LIMIT = 40;
                public static int TURN_CURRENT_LIMIT = 30;

                public static double DRIVE_LOOP_RAMP_RATE = 0.25;
            }

            public class CANCoder{
                public static double FRONT_LEFT_OFFSET = -0.3;
                public static double FRONT_RIGHT_OFFSET = 0.468;
                public static double REAR_LEFT_OFFSET = 0.307;
                public static double REAR_RIGHT_OFFSET = -0.210;
            }

            public static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d( (LENGTHS.TRACK_WIDTH_METERS / 2.0),  (LENGTHS.TRACK_WIDTH_METERS / 2.0)  ),  // Front Left   Translation2D (1,1)   -> (  1 , -1 )
                new Translation2d( (LENGTHS.TRACK_WIDTH_METERS / 2.0),  -(LENGTHS.TRACK_WIDTH_METERS / 2.0) ),  // Front Right  Translation2D (1,-1)  -> (  1 ,  1 )
                new Translation2d( -(LENGTHS.TRACK_WIDTH_METERS / 2.0), (LENGTHS.TRACK_WIDTH_METERS / 2.0)  ),  // Rear  Left   Translation2D (-1,1)  -> ( -1 , -1 )
                new Translation2d( -(LENGTHS.TRACK_WIDTH_METERS / 2.0), -(LENGTHS.TRACK_WIDTH_METERS / 2.0) )   // Rear  Right  Translation2D (-1,-1) -> ( -1 ,  1 )
            ); 

            public static double MAX_ANGULAR_SPEED_RAD = Math.PI * 2;
            public static double MAX_SPEED_METERS = 5;
        }
    }

    public class FieldConstants{
        public static double FIELD_LENGTH = Units.feetToMeters(57.573);
        public static double FIELD_WIDTH = Units.feetToMeters(26.417);

        public static AdvancedPose2D INIT_POSE_BLUE = new AdvancedPose2D(2, 2, Rotation2d.fromDegrees(180));
        public static AdvancedPose2D REEF_CENTER_BLUE = new AdvancedPose2D(4.48945, FIELD_WIDTH / 2, Rotation2d.fromDegrees(0));

        public class AutoAim{
            public static double[] BOUNDARIES = {-180.0, -150.0, -90.0, -30.0, 30.0, 90.0, 150.0, 180.0}; 
            public static Station[] STATIONS = {Station.D, Station.E, Station.F, Station.A, Station.B, Station.C, Station.D};

            public static double REEF_CENTER_TO_ROBOT = 1.3;
            public static double AUTO_TRANSLATION_OFFSET_X = 0.2;
            public static double MANUAL_TRANSLATION_RANGE = 0.4; // Plus Minus .4m 

            public static HashMap<Station, AdvancedPose2D> STATION_BLUE = new HashMap<Station, AdvancedPose2D>(){{
                put(Station.A, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(180), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(0)));
                put(Station.B, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(-120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(60)));
                put(Station.C, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(-60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(120)));
                put(Station.D, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(0), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(180)));
                put(Station.E, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-120)));
                put(Station.F, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-60)));
            }};

            public static HashMap<Station, AdvancedPose2D> STATION_RED = new HashMap<Station, AdvancedPose2D>(){{
                put(Station.A, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(180), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(0)));
                put(Station.B, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(-120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(60)));
                put(Station.C, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(-60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(120)));
                put(Station.D, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(0), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(180)));
                put(Station.E, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-120)));
                put(Station.F, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-60)));
            }};
        }
    }
}
