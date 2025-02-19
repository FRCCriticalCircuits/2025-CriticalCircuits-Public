package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.Constants.DeviceID;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.swerve.swerveIO.GyroIO;
import frc.robot.subsystems.swerve.swerveIO.GyroRedux;
import frc.robot.subsystems.swerve.swerveIO.GyroSim;
import frc.robot.subsystems.swerve.swerveIO.SwerveModule;
import frc.robot.Robot;

public class SwerveSubsystem extends SubsystemBase{
    private static SwerveSubsystem instance;

    // Hardwares
    private SwerveModule frontLeft, frontRight, rearLeft, rearRight;
    private GyroIO gyro;

    // PoseEstimation
    private SwerveDrivePoseEstimator poseEstimator;

    // telemetry
    private Field2d estimateField = new Field2d(), visionEst = new Field2d();
    private StructArrayPublisher<SwerveModuleState> currentSwerveStatePublisher;
    private StructArrayPublisher<SwerveModuleState> desireSwerveStatePublisher;

    // SysId
    public SysIdRoutine routineLinear = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Seconds),
            Volts.of(6),
            Seconds.of(10),
            (state) -> SignalLogger.writeString("sysId-Linear", state.toString())
        ),
        new SysIdRoutine.Mechanism(this::voltageDriveLinear, null, this)
    );

    public SysIdRoutine routineAngular = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.5).per(Seconds),
            Volts.of(6),
            Seconds.of(20),
            (state) -> SignalLogger.writeString("sysId-Angular", state.toString())
        ),
        new SysIdRoutine.Mechanism(this::voltageDriveAngular, null, this)
    );

    public synchronized void voltageDriveLinear(Voltage volt){
        double voltage = volt.magnitude();

        frontLeft.setDriveVoltage(voltage);
        frontRight.setDriveVoltage(voltage);
        rearLeft.setDriveVoltage(voltage);
        rearRight.setDriveVoltage(voltage);
        
        frontLeft.setTurn(0);
        frontRight.setTurn(0);
        rearLeft.setTurn(0);
        rearRight.setTurn(0);
    }

    public synchronized void voltageDriveAngular(Voltage volt){
        double voltage = volt.magnitude();
        
        frontLeft.setDriveVoltage(voltage);
        frontRight.setDriveVoltage(voltage);
        rearLeft.setDriveVoltage(voltage);
        rearRight.setDriveVoltage(voltage);
        
        frontLeft.setTurn(Rotation2d.fromDegrees(135).getRadians());
        frontRight.setTurn(Rotation2d.fromDegrees(45).getRadians());
        rearLeft.setTurn(Rotation2d.fromDegrees(-135).getRadians());
        rearRight.setTurn(Rotation2d.fromDegrees(-45).getRadians());
    }

    public synchronized Command sysIdLinearQuasistatic(SysIdRoutine.Direction direction) {
        return routineLinear.quasistatic(direction);
    }

    public synchronized Command sysIdLinearDynamic(SysIdRoutine.Direction direction) {
        return routineLinear.dynamic(direction);
    }

    public synchronized Command sysIdAngularQuasistatic(SysIdRoutine.Direction direction) {
        return routineAngular.quasistatic(direction);
    }

    public synchronized Command sysIdAngularDynamic(SysIdRoutine.Direction direction) {
        return routineAngular.dynamic(direction);
    }

    private SwerveSubsystem(){
        if(Robot.isReal()) {
            gyro = new GyroRedux();
        }
        else {
            gyro = new GyroSim();
        }
        
        // Swerve Modules
        frontLeft = new SwerveModule(
            DeviceID.DriveBase.FRONT_LEFT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCoder.FRONT_LEFT_OFFSET, 
            DeviceID.DriveBase.FRONT_LEFT_DRIVE_ID, 
            true,
            DeviceID.DriveBase.FRONT_LEFT_TURN_ID,
            true
        );

        frontRight = new SwerveModule(
            DeviceID.DriveBase.FRONT_RIGHT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCoder.FRONT_RIGHT_OFFSET, 
            DeviceID.DriveBase.FRONT_RIGHT_DRIVE_ID, 
            false,
            DeviceID.DriveBase.FRONT_RIGHT_TURN_ID, 
            true
        );

        rearLeft = new SwerveModule(
            DeviceID.DriveBase.REAR_LEFT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCoder.REAR_LEFT_OFFSET, 
            DeviceID.DriveBase.REAR_LEFT_DRIVE_ID, 
            true,
            DeviceID.DriveBase.REAR_LEFT_TURN_ID, 
            true
        );

        rearRight = new SwerveModule(
            DeviceID.DriveBase.REAR_RIGHT_CANCODER_ID, 
            PhysicalConstants.DriveBase.CANCoder.REAR_RIGHT_OFFSET,
            DeviceID.DriveBase.REAR_RIGHT_DRIVE_ID,
            false,
            DeviceID.DriveBase.REAR_RIGHT_TURN_ID,
            true
        );

        resetGyro(-0.5);

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

        SmartDashboard.putData("Estimate Field", estimateField);
        SmartDashboard.putData("Vision Estimate Field", visionEst);
    }

    public synchronized static SwerveSubsystem getInstance(){
        if(instance == null) instance = new SwerveSubsystem();
        return instance;
    }

    public synchronized Rotation2d getGyroRotation2D(){
        return gyro.getGyroRotation2D();
    }

    public synchronized void resetGyro(double yaw){
        gyro.setYaw(yaw);
    }

    /**
     * get position of four modules
     * @return the position in {@link SwerveModulePosition} 
     */
    public synchronized SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            rearLeft.getModulePosition(),
            rearRight.getModulePosition()
        }; 
    }

    /**
     * get vector of four modules
     * @return the position in {@link SwerveModuleState} 
     */
    public synchronized SwerveModuleState[] getSwerveModuleStates() {
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
    public synchronized ChassisSpeeds getChassisSpeeds() {
        return PhysicalConstants.DriveBase.KINEMATICS.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * set the state of four modules with {@link SwerveModuleState} array
     * @param states the desired {@link SwerveModuleState} array
     * @param isOpenLoop true for openLoop control (for drive motor)
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
        if(DriveStationIO.isTest()) desireSwerveStatePublisher.set(states);
    }

    /**
     * set the state of four modules with a {@link ChassisSpeeds} object
     * @param speeds the desired {@link ChassisSpeeds} speed
     * @param isOpenLoop true for openLoop control (for drive motor)
     */
    public synchronized void setModuleStates(ChassisSpeeds speeds, boolean isOpenLoop){
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState states[] = PhysicalConstants.DriveBase.KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states, isOpenLoop);
    }

    /**
     * set the state of four modules with a {@link ChassisSpeeds} object
     * @param states the desired {@link ChassisSpeeds} speed
     * @apiNote closeloop by defualt, uses for pathplanner
     */
    public synchronized void setModuleStates(ChassisSpeeds speeds){
        setModuleStates(speeds, false);
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
        poseEstimator.update(getGyroRotation2D(), getSwerveModulePositions());
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
        poseEstimator.resetPosition(getGyroRotation2D(), getSwerveModulePositions(), pose);
    }

    @Override
    public synchronized void periodic(){
        // Pose Estimator
        updatePoseEstimator();

        // Telemetry
        estimateField.setRobotPose(getPoseEstimate());

        if(DriveStationIO.isTest()) {
            SmartDashboard.putNumber("Gyro", getGyroRotation2D().getRadians());
            currentSwerveStatePublisher.set(getSwerveModuleStates());
        }
    }
}