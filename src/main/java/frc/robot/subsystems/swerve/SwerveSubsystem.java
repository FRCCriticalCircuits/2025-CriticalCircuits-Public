package frc.robot.subsystems.swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.swerve.swerveIO.GyroIO;
import frc.robot.subsystems.swerve.swerveIO.GyroRedux;
import frc.robot.subsystems.swerve.swerveIO.GyroSim;
import frc.robot.subsystems.swerve.swerveIO.Module;
import frc.robot.subsystems.swerve.swerveIO.ModuleIOSim;
import frc.robot.subsystems.swerve.swerveIO.ModuleIOSpark;
import frc.robot.subsystems.swerve.swerveIO.OdometryThread;
import frc.robot.subsystems.swerve.swerveIO.GyroIO.GyroIOInputs;
import frc.robot.Robot;

public class SwerveSubsystem extends SubsystemBase{
    private static SwerveSubsystem instance;

    // Hardwares
    private Module[] modules = new Module[4];

    private GyroIO gyro;
    private GyroIOInputs gyroInputs = new GyroIOInputs();
    private Rotation2d rawGyroRotation = Rotation2d.fromRotations(-0.5);

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    // PoseEstimation
    private SwerveDrivePoseEstimator poseEstimator;

    // telemetry
    private Field2d estimateField = new Field2d(), visionEst = new Field2d();
    private StructArrayPublisher<SwerveModuleState> currentSwerveStatePublisher;
    private StructArrayPublisher<SwerveModuleState> desireSwerveStatePublisher;

    private static SwerveDriveSimulation driveSimulation;

    public static final Lock odometryLock = new ReentrantLock();

    private SwerveSubsystem(){
        driveSimulation = new SwerveDriveSimulation(
            PhysicalConstants.DriveBase.SIMULATION_CONFIG,
            DriveStationIO.isBlue()   ? FieldConstants.INIT_POSE_BLUE 
                                      : FieldConstants.INIT_POSE_BLUE.horizontallyFlip()
        );

        if(Robot.isReal()) {
            gyro = new GyroRedux();

            for (int i = 0; i < 4; i++) {
                modules[i] = new Module(new ModuleIOSpark(i), i);
            }
        }
        else {
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
            gyro = new GyroSim(driveSimulation.getGyroSimulation());

            for(int i = 0; i < 4; i++){
                modules[i] = new Module(new ModuleIOSim(driveSimulation.getModules()[i]), i);
            }
        }

        resetGyro(rawGyroRotation.getRotations());
        
        Timer.delay(0.1); // 100ms delay

        OdometryThread.getInstance().start();

        // Pose Estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.DriveBase.KINEMATICS, 
            rawGyroRotation,
            getSwerveModulePositions(),
            DriveStationIO.isBlue()   ? FieldConstants.INIT_POSE_BLUE 
                                      : FieldConstants.INIT_POSE_BLUE.horizontallyFlip()
        );

        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPoseEstimate, // Robot pose supplier
            this::resetPoseEstimate, // Method to reset odometry (will be called if auto has starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier
            (speeds, feedforwards) -> setModuleStates(speeds), // optionally outputs individual feedforwards
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 10.0, 0.0, 0.5), // Translation Feedback PID constants
                new PIDConstants(4.0, 0.0, 0.0) // Rotation Feedback PID constants
            ),
            config, // The robot configuration
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );

        // Telemetry
        currentSwerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/CurrentState", SwerveModuleState.struct).publish();
        desireSwerveStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates/DesiredState", SwerveModuleState.struct).publish();
    }

    public synchronized static SwerveSubsystem getInstance(){
        if(instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    public synchronized Rotation2d getGyroRotation2D(){
        return rawGyroRotation;
    }

    public synchronized void resetGyro(double yaw){
        gyro.setYaw(yaw);
    }

    /**
     * get position of four modules
     * @return the position in {@link SwerveModulePosition} 
     */
    public synchronized SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * get vector of four modules
     * @return the position in {@link SwerveModuleState} 
     */
    public synchronized SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * get current ChassisSpeeds
     * @return {@link ChassisSpeeds} object
     * @apiNote uses for pathplanner
     */
    public synchronized ChassisSpeeds getChassisSpeeds() {
        return PhysicalConstants.DriveBase.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * set the state of four modules with {@link SwerveModuleState} array
     * @param states the desired {@link SwerveModuleState} array
     * @param isOpenLoop true for openLoop control (for drive motor)
     */
    private void setModuleStates(SwerveModuleState[] states){
        // normalize wheelspeed to make it smaller than the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, PhysicalConstants.DriveBase.MAX_SPEED_METERS);

        // apply speeds to each Swerve Module
        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
        }

        // telemetry
        desireSwerveStatePublisher.set(states);
    }

    /**
     * set the state of four modules with a {@link ChassisSpeeds} object
     * @param speeds the desired {@link ChassisSpeeds} speed
     */
    public synchronized void setModuleStates(ChassisSpeeds speeds){
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState states[] = PhysicalConstants.DriveBase.KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    /**
     * get the position of robot from the {@link SwerveDrivePoseEstimator}
     * @return position in {@link Pose2d}
     */
    public synchronized Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * update the {@link SwerveDrivePoseEstimator}
     */
    public synchronized void updatePoseEstimator() {
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = PhysicalConstants.DriveBase.KINEMATICS.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
    }

    /**
     * update the {@link SwerveDrivePoseEstimator} with Vision Results
     */
    public synchronized void updatePoseEstimator(Pose2d visionEstimatedPose, double timeStampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionEstimatedPose, timeStampSeconds, stdDevs);
        visionEst.setRobotPose(visionEstimatedPose);
    }

    /**
     * reset the position of the {@link SwerveDrivePoseEstimator} 
     * @param pose new position in {@link Pose2d}
     */
    public synchronized void resetPoseEstimate(Pose2d pose) {
        driveSimulation.setSimulationWorldPose(pose);
        poseEstimator.resetPosition(rawGyroRotation, getSwerveModulePositions(), pose);
    }

    @Override
    public synchronized void periodic(){
        odometryLock.lock(); // Prevents odometry updates while reading data
        
        gyro.updateInputs(gyroInputs);

        for (Module module : modules) {
            module.periodic();
        }
        
        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Pose Estimator
        updatePoseEstimator();

        // Telemetry
        estimateField.setRobotPose(getPoseEstimate());

        SmartDashboard.putNumber("Gyro", rawGyroRotation.getRadians());
        currentSwerveStatePublisher.set(getSwerveModuleStates());

        SmartDashboard.putData("Estimate Field", estimateField);
        SmartDashboard.putData("Vision Estimate Field", visionEst);
    }
}