package frc.robot.subsystems.autoaim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.Math.SwerveAimPose;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;

public class AutoAimManager {
    private static AutoAimManager instance;

    public AutoAimSetting setting = new AutoAimSetting(Spot.MID, Level.L1, Mode.INTAKE); // default setting

    private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;
    private Field2d field2d = new Field2d();

    public AutoAimManager(){
        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        thetaController = new PIDController(0, 0, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(0, 0);
        yController.setTolerance(0, 0);
        thetaController.setTolerance(0, 0);
    }

    public static synchronized AutoAimManager getInstance(){
        if(instance == null) instance = new AutoAimManager();
        return instance; 
    }

    public synchronized void runSwerveAutoAim(Translation2d manualTranslation){
        Pose2d currentPose = swerveSubsystem.getPoseEstimate();
        Pose2d targetPose = SwerveAimPose.estimateStationAdvancedPose2D(currentPose, setting, manualTranslation);
        field2d.setRobotPose(targetPose);
        SmartDashboard.putData(field2d);

        double horizontalOutput = xController.calculate(currentPose.getX(), targetPose.getX());
        double verticalOutput = yController.calculate(currentPose.getY(), targetPose.getY());
        double thetaOutput = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        swerveSubsystem.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(horizontalOutput, verticalOutput, thetaOutput, currentPose.getRotation()), false);
    }
}
