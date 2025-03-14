package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.subsystems.AutoAimConstants.PID.*;

import java.util.function.Supplier;

import frc.robot.subsystems.AutoAimConstants.PID.Rotation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoAlignCommand extends Command {
  private SwerveSubsystem swerveSubsystem;
  private ProfiledPIDController translationPIDx;
  private ProfiledPIDController translationPIDy;
  private ProfiledPIDController rotationPID;
  private Supplier<Pose2d> targetSupplier;

  public AutoAlignCommand(Supplier<Pose2d> targetSupplier, SwerveSubsystem s) {
    this.targetSupplier = targetSupplier;
    swerveSubsystem = s;

    // Create PID controllers for x, y, theta
    translationPIDx = new ProfiledPIDController(Translation.kP, Translation.kI, Translation.kD,
        new TrapezoidProfile.Constraints(Translation.maxVelocity, Translation.maxAcceleration));
    translationPIDy = new ProfiledPIDController(Translation.kP, Translation.kI, Translation.kD,
        new TrapezoidProfile.Constraints(Translation.maxVelocity, Translation.maxAcceleration));
    rotationPID = new ProfiledPIDController(Rotation.kP, Rotation.kI, Rotation.kD,
        new TrapezoidProfile.Constraints(Rotation.maxVelocity, Rotation.maxAcceleration));

    addRequirements(s);
  }

  @Override
  public void initialize() {
    translationPIDx.setGoal(0);
    translationPIDy.setGoal(0);
    rotationPID.setGoal(0);
  }

  @Override
  public void execute() {
      swerveSubsystem.getField().getObject("target").setPose(targetSupplier.get());
    
    Pose2d currPose = swerveSubsystem.getPoseEstimate();
    // Get translation vector from current to final pose
    Translation2d dist = currPose
        .getTranslation().minus(targetSupplier.get().getTranslation());
      
    double rotDistCW = currPose.getRotation().getRadians()-targetSupplier.get().getRotation().getRadians();
    double rotDistCCW = (targetSupplier.get().getRotation().getRadians()-currPose.getRotation().getRadians());
    double rotDist = Math.abs(rotDistCCW) <= Math.abs(rotDistCW)? rotDistCW : rotDistCCW;
    rotDist = MathUtil.angleModulus(rotDist);
    
    double tx = translationPIDx.calculate(dist.getX());
    double ty = translationPIDy.calculate(dist.getY());
    double r = rotationPID.calculate(rotDist);

    SmartDashboard.putNumber("PosPIDx", tx);
    SmartDashboard.putNumber("PosPIDy", ty);
    SmartDashboard.putNumber("RotationPID", r);

    ChassisSpeeds c = new ChassisSpeeds(tx, ty, r);

    swerveSubsystem.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(c, currPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.getField().getObject("target").setPose(Pose2d.kZero);
  }
}
