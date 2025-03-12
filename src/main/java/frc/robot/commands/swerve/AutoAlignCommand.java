package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.subsystems.AutoAimConstants.PID.*;

import frc.robot.subsystems.AutoAimConstants.PID.Rotation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoAlignCommand extends Command {
  SwerveSubsystem swerveSubsystem;
  ProfiledPIDController translationPIDx;
  ProfiledPIDController translationPIDy;
  ProfiledPIDController rotationPID;
  Pose2d target;

  public AutoAlignCommand(Pose2d target, SwerveSubsystem s) {
    this.target = target;
    swerveSubsystem = s;
    translationPIDx = new ProfiledPIDController(TranslationX.kP, TranslationX.kI, TranslationX.kD,
        new TrapezoidProfile.Constraints(TranslationX.maxVelocity, TranslationX.maxAcceleration));
    translationPIDy = new ProfiledPIDController(TranslationY.kP, TranslationY.kI, TranslationY.kD,
        new TrapezoidProfile.Constraints(TranslationY.maxVelocity, TranslationY.maxAcceleration));
    rotationPID = new ProfiledPIDController(Rotation.kP, Rotation.kI, Rotation.kD,
        new TrapezoidProfile.Constraints(Rotation.maxVelocity, Rotation.maxAcceleration));

    addRequirements(s);
  }

  @Override
  public void initialize() {
    translationPIDx.setGoal(target.getX());
    translationPIDy.setGoal(target.getY());
    rotationPID.setGoal(target.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    Pose2d currPose = swerveSubsystem.getPoseEstimate();
    double dist = currPose
        .getTranslation()
        .getDistance(target.getTranslation());

    double tx = translationPIDx.calculate(currPose.getX());
    double ty = translationPIDy.calculate(currPose.getY());
    double r = rotationPID.calculate(swerveSubsystem.getGyroRotation2D().getDegrees());

    // vector from start pose to end pose
    // Translation2d startToEnd = target.getTranslation().minus(currPose.getTranslation());
    // startToEnd.inter

    // now have a translational speed t that we want to use, need to separate to components
    // startToEnd.getX()

    ChassisSpeeds c = new ChassisSpeeds(tx, ty, r);

    swerveSubsystem.setModuleStates(c);
  }
}
