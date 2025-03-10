package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.Physical;

public class TeleopDrive extends Command{
    public static TeleopDrive instance;

    public static boolean manualEnable = true;

    private SwerveSubsystem swerveSubsystem;
    
    private Supplier<Double>    verticalSpeed,
                                horizontalSpeed,
                                omegaSpeed,
                                enableRoboRelative;

    private SlewRateLimiter xLimiter,
                            yLimiter,
                            omegaLimiter;

    public TeleopDrive
    (
        Supplier<Double> verticalSpeed,
        Supplier<Double> horizontalSpeed,
        Supplier<Double> omegaSpeed,
        Supplier<Double> enableRoboRelative
    ){
        this.swerveSubsystem = SwerveSubsystem.getInstance();

        this.verticalSpeed = verticalSpeed;
        this.horizontalSpeed = horizontalSpeed;
        this.omegaSpeed = omegaSpeed;
        this.enableRoboRelative = enableRoboRelative;

        this.xLimiter = new SlewRateLimiter(5);
        this.yLimiter = new SlewRateLimiter(5);
        this.omegaLimiter = new SlewRateLimiter(Math.PI);

        addRequirements(swerveSubsystem);
    }

    public static TeleopDrive getInstance
    (
        Supplier<Double> verticalSpeed,
        Supplier<Double> horizontalSpeed,
        Supplier<Double> omegaSpeed,
        Supplier<Double> enableRoboRelative
    ){
        if(instance == null) instance = new TeleopDrive(verticalSpeed, horizontalSpeed, omegaSpeed, enableRoboRelative);
        return instance;
    }

    @Override
    public void execute(){
        if(manualEnable){
            /**
             * Get values from supplier
             * here we swap to the WPI Coordinate System
             */
            double xSpeed = verticalSpeed.get();
            double ySpeed = horizontalSpeed.get();
            double rotSpeed = omegaSpeed.get();
            Boolean relativeDrive = enableRoboRelative.get() > 0.5;
            
            /**
             * Apply Deadbands to Inputs
             */
            xSpeed = Math.abs(xSpeed) > 0.15 ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > 0.15 ? ySpeed : 0.0;
            rotSpeed = Math.abs(rotSpeed) > 0.15 ? rotSpeed : 0.0;

            /**
             * Apply Limiters
             */
            xSpeed = xLimiter.calculate(xSpeed) * Physical.DriveBase.MAX_SPEED_METERS;
            ySpeed = yLimiter.calculate(ySpeed) * Physical.DriveBase.MAX_SPEED_METERS;
            rotSpeed = omegaLimiter.calculate(rotSpeed) * Physical.DriveBase.MAX_ANGULAR_SPEED_RAD;

            ChassisSpeeds speeds = (relativeDrive) ? new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
                                                   : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getGyroRotation2D());

            swerveSubsystem.setModuleStates(speeds);
        }       
    } 

    /**
     * @apiNote always return false to make the command run
     * @apiNote notice that the command will be canceled when disable the robot
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}