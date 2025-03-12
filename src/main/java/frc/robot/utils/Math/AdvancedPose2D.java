package frc.robot.utils.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class AdvancedPose2D extends Pose2d{
    public AdvancedPose2D(Translation2d translation, Rotation2d rotation){
        super(translation, rotation);
    }

    public AdvancedPose2D(double x, double y, Rotation2d rotation){
        super(new Translation2d(x, y), rotation);
    }

    public AdvancedPose2D horizontallyFlip(){
        return new AdvancedPose2D(
            new Translation2d(
                FieldConstants.FIELD_LENGTH - this.getTranslation().getX(),
                this.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(
                (this.getRotation().getDegrees() > 0) ?
                180 - this.getRotation().getDegrees() :
                -(180 + this.getRotation().getDegrees())
            )
        );
    }

    public AdvancedPose2D flip(){
        return new AdvancedPose2D(
            new Translation2d(
                FieldConstants.FIELD_LENGTH - this.getTranslation().getX(),
                FieldConstants.FIELD_WIDTH - this.getTranslation().getY()
            ),
            Rotation2d.fromDegrees(
                (this.getRotation().getDegrees() > 0) ?
                180 - this.getRotation().getDegrees() :
                -(180 + this.getRotation().getDegrees())
            )
        );
    }

    /**
     * Apply a rotated transfromation to the {@link AdvancedPose2D} object
     * @param direction the direction of the translation
     * @param translation Y positive goes front and X positive goes Right
     * @param desireHeading the heading of processed pose
     * @return the transformed {@link AdvancedPose2D} object
     */
    public AdvancedPose2D withVector(Rotation2d direction, Translation2d translation, Rotation2d desireHeading) {
        double x = translation.getX() * direction.getCos() - translation.getY() * direction.getSin() + this.getX();
        double y = translation.getX() * direction.getSin() + translation.getY() * direction.getCos() + this.getY();
        return new AdvancedPose2D(x, y, desireHeading);
    }

    /**
     * apply a robot-relative transformation to the {@link AdvancedPose2D} object
     * @param transformation Y positive goes front and X positive goes Right
     * @return the transformed {@link AdvancedPose2D} object
     */
    public AdvancedPose2D withRobotRelativeTransformation(Translation2d transformation){
        return this.withVector(this.getRotation().minus(Rotation2d.fromDegrees(90)), transformation, this.getRotation()); // minus 90 because 0 axis changes to Y
    }

    public Pose2d getPose2d() {
      return this;
    }
}
