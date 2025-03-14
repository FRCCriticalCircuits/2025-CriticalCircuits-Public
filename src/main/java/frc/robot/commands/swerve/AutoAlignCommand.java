package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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

  // AdvantageScope publisher
  private StructPublisher<Pose2d> targetPublisher;

  public AutoAlignCommand(Supplier<Pose2d> targetSupplier, SwerveSubsystem s) {
    this.targetSupplier = targetSupplier;
    swerveSubsystem = s;

    // Publish target to ascope
    this.targetPublisher = NetworkTableInstance.getDefault().getStructTopic("AutoAlign_Target", Pose2d.struct).publish();

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
    // We are aiming to get to a zero delta
    translationPIDx.setGoal(0);
    translationPIDy.setGoal(0);
    rotationPID.setGoal(0);
  }

  @Override
  public void execute() {
    Pose2d target = targetSupplier.get();
    // Push the target for viewing
    swerveSubsystem.getField().getObject("target").setPose(target);
    targetPublisher.set(target);
    
    Pose2d currPose = swerveSubsystem.getPoseEstimate();
    // Get translation vector from current to final pose
    Translation2d dist = currPose
        .getTranslation().minus(target.getTranslation());
      
    double rotDistCW = currPose.getRotation().getRadians()-target.getRotation().getRadians();
    double rotDistCCW = (target.getRotation().getRadians()-currPose.getRotation().getRadians());
    double rotDist = Math.abs(rotDistCCW) <= Math.abs(rotDistCW)? rotDistCW : rotDistCCW;
    rotDist = MathUtil.angleModulus(rotDist);
    
    // Feed the pid controllers
    double vx = translationPIDx.calculate(dist.getX());
    double vy = translationPIDy.calculate(dist.getY());
    double vr = rotationPID.calculate(rotDist);

    SmartDashboard.putNumber("AutoAlignPID_X", vx);
    SmartDashboard.putNumber("AutoAlignPID_Y", vy);
    SmartDashboard.putNumber("AutoAlignPID_R", vr);

    ChassisSpeeds c = new ChassisSpeeds(vx, vy, vr);

    swerveSubsystem.setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(c, currPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    // Nuke the published poses since auto align isnt running anymore
    swerveSubsystem.getField().getObject("target").setPose(Pose2d.kZero);
    targetPublisher.set(null);
  }
}
