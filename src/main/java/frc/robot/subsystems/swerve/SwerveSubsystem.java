package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhysicalConstants;

public class SwerveSubsystem extends SubsystemBase{
    private static SwerveSubsystem instance;
    
    // Hardwares
    private SwerveModule frontLeft, frontRight, rearLeft, rearRight;
    private Canandgyro gyro;

    // PoseEstimation
    private SwerveDrivePoseEstimator poseEstimator;

    // telemetry
    private Field2d estimateField = new Field2d();
    private StructArrayPublisher<SwerveModuleState> currentSwerveStatePublisher;
    private StructArrayPublisher<SwerveModuleState> desireSwerveStatePublisher;

    private SwerveSubsystem(){
        gyro = new Canandgyro(DeviceID.DriveBase.GYRO_CAN_ID);

        // Swerve Modules
        frontLeft = new SwerveModule(
            DeviceID.DriveBase.FRONT_LEFT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCODER_OFFSET.FRONT_LEFT_OFFSET, 
            DeviceID.DriveBase.FRONT_LEFT_DRIVE_ID, 
            true,
            DeviceID.DriveBase.FRONT_LEFT_TURN_ID,
            true
        );

        frontRight = new SwerveModule(
            DeviceID.DriveBase.FRONT_RIGHT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCODER_OFFSET.FRONT_RIGHT_OFFSET, 
            DeviceID.DriveBase.FRONT_RIGHT_DRIVE_ID, 
            false,
            DeviceID.DriveBase.FRONT_RIGHT_TURN_ID, 
            true
        );

        rearLeft = new SwerveModule(
            DeviceID.DriveBase.REAR_LEFT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCODER_OFFSET.REAR_LEFT_OFFSET, 
            DeviceID.DriveBase.REAR_LEFT_DRIVE_ID, 
            true,
            DeviceID.DriveBase.REAR_LEFT_TURN_ID, 
            true
        );

        rearRight = new SwerveModule(
            DeviceID.DriveBase.REAR_RIGHT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCODER_OFFSET.REAR_RIGHT_OFFSET,
            DeviceID.DriveBase.REAR_RIGHT_DRIVE_ID,
            false,
            DeviceID.DriveBase.REAR_RIGHT_TURN_ID,
            true
        );

        /* Gyro Calibration */
        while(gyro.isCalibrating()) {
            Timer.delay(1);
            Commands.print("Gyro Calibrating").schedule();
        }

        gyro.setYaw(0);
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        rearLeft.resetEncoders();
        rearRight.resetEncoders();
        Timer.delay(0.1); // 100ms delay

        // Pose Estimator
        poseEstimator = new SwerveDrivePoseEstimator(
            PhysicalConstants.DriveBase.KINEMATICS, 
            getGyroRotation2D(),
            getSwerveModulePositions(),
            DriveStationIO.isBlue()   ? FieldConstants.initPose 
                                      : FieldConstants.initPose.horizontallyFlip()
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
                new PIDConstants(0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(0, 0.0, 0.0) // Rotation PID constants
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

    public static SwerveSubsystem getInstance(){
        if(instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    /*
     * Gyro Methods
     */

    public Rotation2d getGyroRotation2D(){
        return gyro.getRotation2d();
    }

    public void resetGyro(double yaw){
        gyro.setYaw(yaw);
    }

    public void resetGyro(){
        gyro.setYaw(0);
    }

    /**
     * get position of four modules
     * @return the position in {@link SwerveModulePosition} 
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            rearLeft.getModulePosition(),
            rearRight.getModulePosition()
        }; 
    }

    /**
     * get velocity & direction of four modules
     * @return the position in {@link SwerveModuleState} 
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
            frontLeft.getModuleState(),
            frontRight.getModuleState(),
            rearLeft.getModuleState(),
            rearRight.getModuleState()
        };
    }

    /**
     * get current ChassisSpeeds
     * @return {@link ChassisSpeeds} object
     * @apiNote uses for pathplanner
     */
    public ChassisSpeeds getChassisSpeeds() {
        return PhysicalConstants.DriveBase.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * set the state of four modules with {@link SwerveModuleState} array
     * @param states the desired {@link SwerveModuleState} array
     * @param isOpenLoop true for duty cycle
     */
    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop){
        // normalize wheelspeed to make it smaller than the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(states, PhysicalConstants.DriveBase.MAX_SPEED_METERS);

        // apply speeds to each Swerve Module
        frontLeft.setState(states[0], isOpenLoop);
        frontRight.setState(states[1], isOpenLoop);
        rearLeft.setState(states[2], isOpenLoop);
        rearRight.setState(states[3], isOpenLoop);

        // telemetry
        desireSwerveStatePublisher.set(states);
    }

    /**
     * set the state of four modules with a {@link ChassisSpeeds} object
     * @param speeds the desired {@link ChassisSpeeds} speed
     * @param isOpenLoop true for duty cycle
     */
    public void setModuleStates(ChassisSpeeds speeds, boolean isOpenLoop){
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState states[] = PhysicalConstants.DriveBase.KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states, isOpenLoop);
    }

    /**
     * set the state of four modules with a {@link ChassisSpeeds} object
     * @param states the desired {@link ChassisSpeeds} speed
     * @apiNote uses for pathpalnner, CloseLoop by defualt
     */
    public void setModuleStates(ChassisSpeeds speeds){
        setModuleStates(speeds, false);
    }

    /**
     * get the position of robot from the {@link SwerveDrivePoseEstimator}
     * @return position in {@link Pose2d}
     */
    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * update the {@link SwerveDrivePoseEstimator}
     */
    public void updatePoseEstimator() {
        poseEstimator.update(getGyroRotation2D(), getSwerveModulePositions());
    }

    /**
     * reset the position of the {@link SwerveDrivePoseEstimator} 
     * @param pose new position in {@link Pose2d}
     */
    public void resetPoseEstimate(Pose2d pose) {
        poseEstimator.resetPosition(getGyroRotation2D(), getSwerveModulePositions(), pose);
    }

    @Override
    public void periodic(){
        // Pose Estimator
        updatePoseEstimator();

        // Telemetry
        estimateField.setRobotPose(getPoseEstimate());
        SmartDashboard.putData("Estimate Field", estimateField);
        SmartDashboard.putNumber("Gyro", gyro.getYaw() * Math.PI * 2);

        currentSwerveStatePublisher.set(getSwerveModuleStates());
    }

    boolean simulationInitRunned = false;

    public void simulationInit(){
    }

    @Override
    public void simulationPeriodic() {
        if(!simulationInitRunned){
            simulationInit();
            simulationInitRunned = true;
        }
    }
}