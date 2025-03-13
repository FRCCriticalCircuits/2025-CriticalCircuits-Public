package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.Math.AdvancedPose2D;

public class AutoAimConstants {
  public static class PID {
    public static class TranslationX {
      public static final double kP = 3;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double maxVelocity = 1.0;
      public static final double maxAcceleration = 2.5;
    }
    public static class TranslationY {
      public static final double kP = 3;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double maxVelocity = 1.0;
      public static final double maxAcceleration = 2.5;
    }

    public static class Rotation {
      public static final double kP = 0.5;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double maxVelocity = 1.0;
      public static final double maxAcceleration = 1.0;
    }
  }
  
  // inches
  public static final double REEF_EDGE_TO_CENTER = Units.inchesToMeters(32.75);
  // From side to side
  public static final double ROBOT_BUMPER_WIDTH = Units.inchesToMeters(33.75);
  //
  public static final double REEF_CENTER_TO_ROBOT_CENTER = REEF_EDGE_TO_CENTER + ROBOT_BUMPER_WIDTH / 2;
  public static final double REEF_POST_OFFSET = Units.inchesToMeters(6.5);

  public static AdvancedPose2D REEF_CENTER_BLUE = new AdvancedPose2D(4.48945, FieldConstants.FIELD_WIDTH / 2,
      Rotation2d.fromDegrees(0));
  public static AdvancedPose2D REEF_CENTER_RED = new AdvancedPose2D(4.48945, FieldConstants.FIELD_LENGTH / 2,
      Rotation2d.fromDegrees(0)).horizontallyFlip();

  public static AdvancedPose2D[] REEF_POSES_BLUE = {
      // AB
      REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(0), new Translation2d(-REEF_CENTER_TO_ROBOT_CENTER, 0),
          Rotation2d.fromDegrees(0)),
      // CD
      REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(120), new Translation2d(-REEF_CENTER_TO_ROBOT_CENTER, 0),
          Rotation2d.fromDegrees(120)),
      // EF
      REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(60), new Translation2d(-REEF_CENTER_TO_ROBOT_CENTER, 0),
          Rotation2d.fromDegrees(60)),
      // GH
      REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(0), new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0),
          Rotation2d.fromDegrees(180)),
      // IJ
      REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(60), new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0),
          Rotation2d.fromDegrees(-120)),
      // KL
      REEF_CENTER_BLUE.withVector(Rotation2d.fromDegrees(120), new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0),
          Rotation2d.fromDegrees(-60))
  };

  public static AdvancedPose2D[] REEF_POSES_RED = {
      REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(0),
          new Translation2d(-REEF_CENTER_TO_ROBOT_CENTER, 0), Rotation2d.fromDegrees(0)),
      REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(-120),
          new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0), Rotation2d.fromDegrees(60)),
      REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(-60),
          new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0), Rotation2d.fromDegrees(120)),
      REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(0),
          new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0), Rotation2d.fromDegrees(180)),
      REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(60),
          new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0), Rotation2d.fromDegrees(-120)),
      REEF_CENTER_BLUE.horizontallyFlip().withVector(Rotation2d.fromDegrees(120),
          new Translation2d(REEF_CENTER_TO_ROBOT_CENTER, 0), Rotation2d.fromDegrees(-60))
  };
}
