package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.subsystems.AutoAimConstants.PID.*;

import java.util.function.Supplier;

import frc.robot.subsystems.AutoAimConstants.PID.Rotation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoAlignCommand extends Command {
  SwerveSubsystem swerveSubsystem;
  ProfiledPIDController translationPIDx;
  ProfiledPIDController translationPIDy;
  ProfiledPIDController rotationPID;
  Supplier<Pose2d> targetSupplier;
  Pose2d target;

  public AutoAlignCommand(Supplier<Pose2d> targetSupplier, SwerveSubsystem s) {
    this.targetSupplier = targetSupplier;
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
    target = targetSupplier.get();
    translationPIDx.setGoal(0);
    translationPIDy.setGoal(0);
    rotationPID.setGoal(target.getRotation().getRadians());
  }

  @Override
  public void execute() {
      swerveSubsystem.getField().getObject("target").setPose(target);
    
    Pose2d currPose = swerveSubsystem.getPoseEstimate();
    Translation2d dist = currPose
        .getTranslation().minus(target.getTranslation());

    double tx = translationPIDx.calculate(dist.getX());
    double ty = translationPIDy.calculate(dist.getY());
    double r = rotationPID.calculate(currPose.getRotation().getRadians());

    SmartDashboard.putNumber("PosPIDx", tx);
    SmartDashboard.putNumber("PosPIDy", ty);
    SmartDashboard.putNumber("RotationPID", r);

    // vector from start pose to end pose
    // Translation2d startToEnd = target.getTranslation().minus(currPose.getTranslation());
    // startToEnd.inter

    // now have a translational speed t that we want to use, need to separate to components
    // startToEnd.getX()

    // ChassisSpeeds c = new ChassisSpeeds(tx, ty, Units.degreesToRadians(r));
    ChassisSpeeds c = new ChassisSpeeds(tx, ty, r);

    swerveSubsystem.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(c, currPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.getField().getObject("target").setPose(Pose2d.kZero);
  }
}
