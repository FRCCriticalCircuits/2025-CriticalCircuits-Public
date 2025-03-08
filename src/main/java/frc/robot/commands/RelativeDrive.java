package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.Physical;

public class RelativeDrive extends Command {

    private SwerveSubsystem swerveSubsystem;

    private double speed;

    public RelativeDrive(
            double speed) {
        this.swerveSubsystem = SwerveSubsystem.getInstance();

        this.speed = speed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        double ySpeed = speed;
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, ySpeed, 0, swerveSubsystem.getGyroRotation2D());
        swerveSubsystem.setModuleStates(speeds);
    }

    /**
     * @apiNote always return false to make the command run
     * @apiNote notice that the command will be canceled when disable the robot
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, swerveSubsystem.getGyroRotation2D());
        swerveSubsystem.setModuleStates(speeds);
    }
}