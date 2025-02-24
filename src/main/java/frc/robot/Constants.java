package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Math.AdvancedPose2D;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;
import frc.robot.utils.structures.DataStrcutures.Station;

public class Constants {
    public static AutoAimSetting DEFAULT_SETTING = new AutoAimSetting(Spot.MID, Level.L1, Mode.CORAL_PLACE);
    public static int SAMPLE_NUM = 20;

    public class KeyBinding{
        public static int GYRO_RESET = 8; // Menu
    }

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

        public class Elevator{
            public static final int ELEVATOR_LEFT_ID = 21;
            public static final int ELEVATOR_RIGHT_ID = 22;
        }

        public class Angler{
            public static final int ANGLER_ID = 25;
            public static final int HATCHER_ID = 26;
            public static final int INTAKE_ID = 27;
        }

        public class Sensor{
            public static final int CORAL_SENSOR = 23;
            public static final int ALGAE_SENSOR = 24;
            public static final int ANGLER_ENCODER = 4;
        }

        public static final int CLIMBER_ID = 29;
    }

    public class TunedConstants{
        public class DriveBase{
            public static double DRIVE_PID_P = 0.1;
            public static double DRIVE_PID_I = 0;

            public static double DRIVE_FEED_FORWARD_KS = 0.015;
            public static double DRIVE_FEED_FORWARD_KV = 0.17;
            public static double DRIVE_FEED_FORWARD_KA = 0;

            public static double TURN_PID_P = 0.3;
            public static double TURN_PID_I = 0;
            public static double TURN_PID_D = 0;

            public class Simulation{
                public static double DRIVE_PID_P = 3.0;
                public static double DRIVE_PID_I = 0;
                public static double DRIVE_PID_D = 0;
    
                public static double DRIVE_FEED_FORWARD_KS = 0.017;
                public static double DRIVE_FEED_FORWARD_KV = 0.7;
                public static double DRIVE_FEED_FORWARD_KA = 0;
    
                public static double TURN_PID_P = 30;
                public static double TURN_PID_I = 0;
                public static double TURN_PID_D = 0;
            }
        }

        public class Elevator{
            public static double ELEVATOR_PID_P = 50.0;
            public static double ELEVATOR_PID_I = 0;
            public static double ELEVATOR_PID_D = 2.0;

            public static double ELEVATOR_FEED_FORWARD_KS = 0.16;
            public static double ELEVATOR_FEED_FORWARD_KV = 1.6;
            public static double ELEVATOR_FEED_FORWARD_KA = 0.12;

            public static double ELEVATOR_MAX_VELOCITY = 5.0;
            public static double ELEVATOR_MAX_ACCELERATION = 35.0;
        }

        public class Arm{
            public static double ARM_PID_P = 50.0;
            public static double ARM_PID_I = 0;
            public static double ARM_PID_D = 2.0;

            public static double ARM_FEED_FORWARD_KS = 0.16;
            public static double ARM_FEED_FORWARD_KV = 1.6;
            public static double ARM_FEED_FORWARD_KA = 0.12;

            public static double ARM_MAX_VELOCITY = 5.0;
            public static double ARM_MAX_ACCELERATION = 35.0;
        }
    }

    public class PhysicalConstants {
        public static double NOMINAL_VOLTAGE = 12.0;
        public static double ODOMETRY_FREQUENCY = 100.0;
        
        public class DriveBase {
            // drivetrain simulation configuration
            public static DriveTrainSimulationConfig SIMULATION_CONFIG = DriveTrainSimulationConfig.Default()
                // Specify gyro type (for realistic gyro drifting and error simulation)
                .withGyro(COTS.ofPigeon2())
                // Specify swerve module (for realistic swerve dynamics)
                .withSwerveModule(
                COTS.ofMark4i(
                        DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                        DCMotor.getNEO(1), // Steer motor is a NEO
                        1.3, // estimate COF for 3D-printed Wheels
                        3 // L3 Gear Ratio
                    )
                )
                // Configures the track length and track width (spacing between swerve modules)
                .withTrackLengthTrackWidth(Meters.of(0.554), Meters.of(0.554))
                // Configures the bumper size (dimensions of the robot bumper)
                .withBumperSize(Meters.of(0.864), Meters.of(0.864));

            public class LENGTHS{
                public static double TRACK_WIDTH_METERS = 0.55245;

                public static double DRIVE_WHEEL_DIAMETER_INCHES = 4;
                public static double DRIVE_WHEEL_DIAMETER_METERS = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER_INCHES);
                public static double DRIVE_WHEEL_RADIUS_METERS = DRIVE_WHEEL_DIAMETER_METERS / 2;
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
                public static double REAR_LEFT_OFFSET = 0.292;
                public static double REAR_RIGHT_OFFSET = -0.210;
            }

            public static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d( (LENGTHS.TRACK_WIDTH_METERS / 2.0),  (LENGTHS.TRACK_WIDTH_METERS / 2.0)  ),  // Front Left   Translation2D (1,1)   -> (  1 , -1 )
                new Translation2d( (LENGTHS.TRACK_WIDTH_METERS / 2.0),  -(LENGTHS.TRACK_WIDTH_METERS / 2.0) ),  // Front Right  Translation2D (1,-1)  -> (  1 ,  1 )
                new Translation2d( -(LENGTHS.TRACK_WIDTH_METERS / 2.0), (LENGTHS.TRACK_WIDTH_METERS / 2.0)  ),  // Rear  Left   Translation2D (-1,1)  -> ( -1 , -1 )
                new Translation2d( -(LENGTHS.TRACK_WIDTH_METERS / 2.0), -(LENGTHS.TRACK_WIDTH_METERS / 2.0) )   // Rear  Right  Translation2D (-1,-1) -> ( -1 ,  1 )
            ); 

            public static double MAX_ANGULAR_SPEED_RAD = Math.PI * 2;
            public static double MAX_SPEED_METERS = 5.500;
        }

        public class Elevator{
            public static double ELEVATOR_GEAR_RATIO = 15.0;

            public class CurrentLimits{
                public static int ELEVATOR_CURRENT_LIMIT = 20;
                public static int ANGLER_CURRENT_LIMIT = 20;
                public static int ROLLER_CURRENT_LIMIT = 10;
            }
        }

        public class Arm{
            public static Rotation2d OFFSET_CENTER_GRAVITY = Rotation2d.fromDegrees(-0.05);
            public static double ENCODER_ZERO_OFFSET = 0.0;
        }
    }

    public class FieldConstants{
        // 1.3147m from the front of robot to surface of reef
        // suppose to be 1cm away from reef

        public static double FIELD_LENGTH = 17.548;
        public static double FIELD_WIDTH = 8.052;

        public static AdvancedPose2D INIT_POSE_BLUE = new AdvancedPose2D(8, 7.620, Rotation2d.fromDegrees(180));
        public static AdvancedPose2D REEF_CENTER_BLUE = new AdvancedPose2D(4.48945, FIELD_WIDTH / 2, Rotation2d.fromDegrees(0));

        public class AutoAim{
            public static double REEF_CENTER_TO_ROBOT = 1.45;
            public static double CORAL_STATION_TO_ROBOT = Units.inchesToMeters(27) / 2;
            public static double AUTO_TRANSLATION_OFFSET_X = 0.2; // Shift for L/R coral
            public static double MANUAL_TRANSLATION_RANGE = 0.4; // Plus Minus .4m 

            public static HashMap<Station, AdvancedPose2D> STATION_BLUE = new HashMap<Station, AdvancedPose2D>(){{
                put(Station.A, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(0), new Translation2d(-REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(0)));
                put(Station.B, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(-120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(60)));
                put(Station.C, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(-60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(120)));
                put(Station.D, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(0), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(180)));
                put(Station.E, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-120)));
                put(Station.F, REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-60)));
            }};

            public static HashMap<Station, AdvancedPose2D> STATION_RED = new HashMap<Station, AdvancedPose2D>(){{
                put(Station.A, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(0), new Translation2d(-REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(0)));
                put(Station.B, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(-120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(60)));
                put(Station.C, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(-60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(120)));
                put(Station.D, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(0), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(180)));
                put(Station.E, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(60), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-120)));
                put(Station.F, REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(120), new Translation2d(REEF_CENTER_TO_ROBOT, 0), Rotation2d.fromDegrees(-60)));
            }};

            public static AdvancedPose2D CORAL_STATION_A = new AdvancedPose2D(new Translation2d(0.836168, 0.6334625), null).withVector(Rotation2d.fromDegrees(54), new Translation2d(CORAL_STATION_TO_ROBOT, 0), Rotation2d.fromDegrees(-126));
            public static AdvancedPose2D CORAL_STATION_B = new AdvancedPose2D(new Translation2d(0.836168, 7.4185375), null).withVector(Rotation2d.fromDegrees(-54), new Translation2d(CORAL_STATION_TO_ROBOT, 0), Rotation2d.fromDegrees(126));
            public static AdvancedPose2D CORAL_STATION_C = CORAL_STATION_A.horizontallyFlip();
            public static AdvancedPose2D CORAL_STATION_D = CORAL_STATION_B.horizontallyFlip();
        }
    }
}
