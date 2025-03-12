package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.AutoAim;
import frc.robot.subsystems.elevator.RollerSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.utils.DriveStationIO.DriveStationIO;
import frc.robot.utils.Math.AdvancedPose2D;
import frc.robot.utils.structures.AutoAimSetting;
import frc.robot.utils.structures.DataStrcutures.Level;
import frc.robot.utils.structures.DataStrcutures.Mode;
import frc.robot.utils.structures.DataStrcutures.Spot;
import frc.robot.utils.web.WebServer;

public class AutoAimManager {
  public enum Reef {
    LEFT, CENTER, RIGHT
  }

  private static AutoAimManager instance;
  private AutoAimSetting settings = Constants.DEFAULT_SETTING;

  private Notifier notifier = new Notifier(this::updateValues);

  private Command command;

  private Supplier<Double> LTSupplier, RTSupplier;

  private Pose2d targetPose = new Pose2d();
  private double targetAngle = 0;

  private StructPublisher<Pose2d> autoAimPositionPublisher;

  private PathConstraints constraints = new PathConstraints(
      .9,
      3,
      Math.PI / 4,
      Math.PI / 2);

  public AutoAimManager(Supplier<Double> LTSupplier, Supplier<Double> RTSupplier) {
    this.LTSupplier = LTSupplier;
    this.RTSupplier = RTSupplier;

    notifier.setName("Autoaim Thread");
    notifier.startPeriodic(0.02);

    autoAimPositionPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/AutoAim/estimatedPosition", Pose2d.struct).publish();

    // for (int i = 0; i < 6; i++) {
    // SwerveSubsystem.getInstance().m_field.getObject(String.valueOf(i)).setPose(
    //   getNearestReef(AutoAimConstants.REEF_POSES_BLUE[i], Reef.RIGHT)
    // );
    // }

    Pose2d temp = new Pose2d(5.9, 6.24, Rotation2d.kZero);
    SwerveSubsystem.getInstance().m_field.getObject("temp").setPose(temp);
    // System.out.println(getNearestReef(temp).getPose2d());
    // SwerveSubsystem.getInstance().m_field.getObject("target").setPose(getNearestReef(temp).getPose2d());

  }

  public synchronized static AutoAimManager getInstance() {
    if (instance == null)
      throw new NullPointerException("AutoAimManager is not initialized");
    return instance;
  }

  public synchronized static AutoAimManager getInstance(Supplier<Double> LTSupplier, Supplier<Double> RTSupplier) {
    if (instance == null)
      instance = new AutoAimManager(LTSupplier, RTSupplier);
    return instance;
  }

  /**
   * Find the cloest coral station based on current Pos
   * 
   * @param currentPos current {@link Translation2d} for the robot
   * @return the pose for target coral station
   */
  private AdvancedPose2D nearestCoralStation(Translation2d currentPos) {
    if (DriveStationIO.getAlliance() == Alliance.Blue) {
      if (currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_A.getTranslation()) > currentPos
          .getDistance(FieldConstants.AutoAim.CORAL_STATION_B.getTranslation()))
        return FieldConstants.AutoAim.CORAL_STATION_B;
      else
        return FieldConstants.AutoAim.CORAL_STATION_A;
    } else {
      if (currentPos.getDistance(FieldConstants.AutoAim.CORAL_STATION_C.getTranslation()) > currentPos
          .getDistance(FieldConstants.AutoAim.CORAL_STATION_D.getTranslation()))
        return FieldConstants.AutoAim.CORAL_STATION_D;
      else
        return FieldConstants.AutoAim.CORAL_STATION_C;
    }
  }

  /**
   * Find the nearest reef side (center)
   * 
   * @param currentPose Current robot pose
   * @return Reef side pose constant
   */
  public AdvancedPose2D getNearestReef(Pose2d currentPose, Reef pos) {
    AdvancedPose2D targetPose = null;
    double minDist = Double.MAX_VALUE;
    int idx = -1;

    // for (AdvancedPose2D pose : AutoAimConstants.REEF_POSES) {
    //   // Find distance between robot and reef posts
    //   double tmpDist = pose.getTranslation().getDistance(currentPose.getTranslation());
    //   // check all posts to find the closest one
    //   if (tmpDist < minDist) {
    //     targetPose = pose;
    //     minDist = tmpDist;
    //   }
    // }

    List<Pose2d> reefPoses;
    AdvancedPose2D reefCenter;

    if (DriveStationIO.getAlliance() == Alliance.Blue) {
      reefPoses = Arrays.asList(AutoAimConstants.REEF_POSES_BLUE);
      reefCenter = AutoAimConstants.REEF_CENTER_BLUE;
    } else {
      // red alliance
      reefPoses = Arrays.asList(AutoAimConstants.REEF_POSES_RED);
      reefCenter = AutoAimConstants.REEF_CENTER_BLUE;
    }

    Pose2d pose = currentPose.nearest(reefPoses);
    targetPose = new AdvancedPose2D(pose.getTranslation(), pose.getRotation());

    // set the target angle
    this.targetAngle = idx * 60;

    switch (pos) {
      case LEFT:
        targetPose = reefCenter.withVector(Rotation2d.fromDegrees(60 * idx), new Translation2d(-AutoAimConstants.REEF_CENTER_TO_ROBOT_CENTER, AutoAimConstants.REEF_POST_OFFSET), 
        Rotation2d.fromDegrees(60 * idx));
        break;
      case RIGHT:
        targetPose = reefCenter.withVector(Rotation2d.fromDegrees(60 * idx), new Translation2d(-AutoAimConstants.REEF_CENTER_TO_ROBOT_CENTER, -AutoAimConstants.REEF_POST_OFFSET), 
        Rotation2d.fromDegrees(60 * idx));
        break;
      case CENTER:
        // No change
        break;
    }

    return targetPose;
  }

  /**
   * estimate the station based on current heading
   * 
   * @param setting           Autoaim Setting Includes {@link Spot},
   *                          {@link Level}, {@link Mode}
   * @param manualTranslation manual translation distence from the origin aim pos
   * @return {@link AdvancedPose2D} for pathplanning
   */
  private AdvancedPose2D estimateAimPos(AutoAimSetting setting, Translation2d manualTranslation) {
    Pose2d currentPos = SwerveSubsystem.getInstance().getPoseEstimate();

    if (setting.getMode() == Mode.CORAL_INTAKE) {
      return nearestCoralStation(currentPos.getTranslation());
    } else if (setting.getMode() == Mode.CORAL_PLACE) {
      AdvancedPose2D aimPose = getNearestReef(currentPos, Reef.CENTER);

      if (setting.getSpot() == Spot.L) {
        return aimPose.withRobotRelativeTransformation(new Translation2d(
            -FieldConstants.AutoAim.AUTO_TRANSLATION + FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET, 0));
      } else if (setting.getSpot() == Spot.R) {
        return aimPose.withRobotRelativeTransformation(new Translation2d(
            FieldConstants.AutoAim.AUTO_TRANSLATION + FieldConstants.AutoAim.AUTO_TRANSLATION_OFFSET, 0));
      } else {
        return aimPose.withRobotRelativeTransformation(manualTranslation);
      }
    } else {
      AdvancedPose2D aimPose = getNearestReef(currentPos, Reef.CENTER);
      return aimPose.withRobotRelativeTransformation(manualTranslation);
    }
  }

  /**
   * a perodic funtion (0.05s) updates values from WebSocket, save/send Estimate
   * Target Pose
   */
  private void updateValues() {
    // get the nearest coral station.
    targetPose = estimateAimPos(
        this.settings,
        new Translation2d( // DAMN THATS SMART RAYMOND
            FieldConstants.AutoAim.MANUAL_TRANSLATION_RANGE * (LTSupplier.get() - RTSupplier.get()),
            0));

    autoAimPositionPublisher.set(targetPose);
  }

  /**
   * get the PathFinding Command based on current settings
   * 
   * @return the {@link Command} to execute
   */
  public synchronized Command pathFindCommand() {
    updateValues();

    // push the wanted pose to the Field2d for viewing
    SwerveSubsystem.getInstance().m_field.getObject("Target").setPose(targetPose);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    this.command = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0 // Goal end velocity in meters/sec
    );

    return new PrintCommand("autoaim");
  }

  public synchronized Command profiledPidCommand() {
    return null;
  }

  public synchronized boolean isFinished() {
    return this.command.isFinished();
  }

  public synchronized void cancle() {
    if (this.command != null)
      this.command.cancel();
  }

  public synchronized AutoAimSetting getSetting() {
    return settings;
  }

  public synchronized void updateSetting(AutoAimSetting desireSetting) {
    this.settings = desireSetting;
  }

  public synchronized void updateSpot(Spot spot) {
    updateSetting(this.settings.withSpot(spot));
  }

  public synchronized void updateMode(Mode mode) {
    updateSetting(this.settings.withMode(mode));

    LEDSubsystem ledSubsystem = LEDSubsystem.getInstance();

    switch (mode.value) {
      case 0:
        ledSubsystem.setColor(Color.kRed);
        break;
      case 1:
        ledSubsystem.setColor(Color.kBlue);
        break;
      case 2:
        ledSubsystem.setColor(Color.kGreen);
        break;
    }
  }

  public synchronized void updateLevel(Level level) {
    updateSetting(this.settings.withLevel(level));
  }

  public Mode getMode() {
    return this.settings.getMode();
  }
}
