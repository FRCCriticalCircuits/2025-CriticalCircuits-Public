package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.Physical;

public class TeleopDrive extends Command {
  public static TeleopDrive instance;
  public static boolean manualEnable = true;
  // xy speed offsets for the robot
  private static double xOffset;
  private static double yOffset;

  private SwerveSubsystem swerveSubsystem;

  private Supplier<Double> verticalSpeed,
      horizontalSpeed,
      omegaSpeed,
      scalingFactorA,
      scalingFactorB;
  private SlewRateLimiter xLimiter,
      yLimiter,
      omegaLimiter;

  public TeleopDrive(
      Supplier<Double> verticalSpeed,
      Supplier<Double> horizontalSpeed,
      Supplier<Double> omegaSpeed,
      Supplier<Double> scalingFactorA,
      Supplier<Double> scalingFactorB) {
    this.swerveSubsystem = SwerveSubsystem.getInstance();

    this.verticalSpeed = verticalSpeed;
    this.horizontalSpeed = horizontalSpeed;
    this.omegaSpeed = omegaSpeed;
    this.scalingFactorA = scalingFactorA;
    this.scalingFactorB = scalingFactorB;

    this.xLimiter = new SlewRateLimiter(5);
    this.yLimiter = new SlewRateLimiter(5);
    this.omegaLimiter = new SlewRateLimiter(Math.PI);

    this.xOffset = 0;
    this.yOffset = 0;

    addRequirements(swerveSubsystem);
  }

  public static TeleopDrive getInstance(
      Supplier<Double> verticalSpeed,
      Supplier<Double> horizontalSpeed,
      Supplier<Double> omegaSpeed,
      Supplier<Double> scalingFactorA,
      Supplier<Double> scalingFactorB) {
    if (instance == null)
      instance = new TeleopDrive(verticalSpeed, horizontalSpeed, omegaSpeed, scalingFactorA, scalingFactorB);
    return instance;
  }

  @Override
  public void execute() {
    if (manualEnable) {
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
      xSpeed = xLimiter.calculate(xSpeed) * Physical.DriveBase.MAX_SPEED_METERS;
      ySpeed = yLimiter.calculate(ySpeed) * Physical.DriveBase.MAX_SPEED_METERS;
      rotSpeed = omegaLimiter.calculate(rotSpeed) * Physical.DriveBase.MAX_ANGULAR_SPEED_RAD;

      ChassisSpeeds speeds;

      // if (RobotContainer.getDriveController().povLeft().getAsBoolean()) {
      // speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, -0.2, 0,
      // swerveSubsystem.getGyroRotation2D());
      // } else if (RobotContainer.getDriveController().povRight().getAsBoolean()) {
      // speeds = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0.2, 0,
      // swerveSubsystem.getGyroRotation2D());
      // }
      // else {
      // speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
      // swerveSubsystem.getGyroRotation2D());
      // }

      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, swerveSubsystem.getGyroRotation2D());
      
      // FIXME: remove the offsets if they mess up
      // Add in the speed offsets and clamp to drivebase max speed
      // speeds.vxMetersPerSecond = Math.max(speeds.vxMetersPerSecond + xOffset, Physical.DriveBase.MAX_SPEED_METERS);
      // speeds.vyMetersPerSecond = Math.max(speeds.vyMetersPerSecond + yOffset, Physical.DriveBase.MAX_SPEED_METERS);
      swerveSubsystem.setModuleStates(speeds);

      // SmartDashboard.putNumber("Requested x", speeds.vxMetersPerSecond);
      // SmartDashboard.putNumber("Requested y", speeds.vyMetersPerSecond);

      // SmartDashboard.putNumber("True x");
      // SmartDashboard.putNumber("True y", speeds.vyMetersPerSecond);
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

  /**
   * Set the current x robot-relative speed offset
   * @param offset velocity in m/s
   */
  public static void setRelativeXSpeedOffset(double offset) {
    TeleopDrive.xOffset = offset;
  }

  /**
   * Set the current y robot-relative speed offset
   * @param offset velocity in m/s
   */
  public static void setRelativeYSpeedOffset(double offset) {
    TeleopDrive.yOffset = offset;
  }
}