package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.PhysicalConstants;

public class teleopDrive extends Command{
    private SwerveSubsystem swerveSubsystem;
    
    private Supplier<Double>    verticalSpeed,
                                horizontalSpeed,
                                omegaSpeed,
                                scalingFactorA,
                                scalingFactorB;
    private SlewRateLimiter xLimiter,
                            yLimiter,
                            omegaLimiter;

    public teleopDrive
    (
        Supplier<Double> verticalSpeed,
        Supplier<Double> horizontalSpeed,
        Supplier<Double> omegaSpeed,
        Supplier<Double> scalingFactorA,
        Supplier<Double> scalingFactorB
    ){
        this.swerveSubsystem = SwerveSubsystem.getInstance();

        this.verticalSpeed = verticalSpeed;
        this.horizontalSpeed = horizontalSpeed;
        this.omegaSpeed = omegaSpeed;
        this.scalingFactorA = scalingFactorA;
        this.scalingFactorB = scalingFactorB;

        this.xLimiter = new SlewRateLimiter(3);
        this.yLimiter = new SlewRateLimiter(3);
        this.omegaLimiter = new SlewRateLimiter(3);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute(){
        /**
         * Get values from supplier
         * here we swap to the WPI Coordinate System
         */
        double xSpeed = verticalSpeed.get();
        double ySpeed = horizontalSpeed.get();
        double rotSpeed = omegaSpeed.get();
        double factorA = scalingFactorA.get();
        double factorB = scalingFactorB.get();
        
        /**
         * Apply Deadbands to Inputs
         */
        xSpeed = Math.abs(xSpeed) > 0.15 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.15 ? ySpeed : 0.0;
        rotSpeed = Math.abs(rotSpeed) > 0.15 ? rotSpeed : 0.0;
        factorA = (factorA > 0.3) ? factorA : 0.3;
        factorB = (factorB > 0.3) ? factorB : 0.3;
        
        /**
         * Apply Speed Factors
         */
        xSpeed *= (factorA + factorB) * 0.5;
        ySpeed *= (factorA + factorB) * 0.5;

        /**
         * Apply Limiters
         */
        xSpeed = xLimiter.calculate(xSpeed) * PhysicalConstants.DriveBase.MAX_SPEED_METERS;
        ySpeed = yLimiter.calculate(ySpeed) * PhysicalConstants.DriveBase.MAX_SPEED_METERS;
        rotSpeed = omegaLimiter.calculate(rotSpeed) * PhysicalConstants.DriveBase.MAX_ANGULAR_SPEED_RAD;

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getGyroRotation2D());
        swerveSubsystem.setModuleStates(speeds, false);
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