package frc.robot.utils.Math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.utils.DataStrcutures.Station;

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

    public AdvancedPose2D withVector(Rotation2d direction, Translation2d translation, Rotation2d desireHeading) {
        double x = translation.getX() * direction.getCos() - translation.getY() * direction.getSin();
        double y = translation.getX() * direction.getSin() + translation.getY() * direction.getCos();
        x += this.getX();
        y += this.getY();
        return new AdvancedPose2D(x, y, desireHeading);
    }

    public AdvancedPose2D withRobotRelativeTransformation(Translation2d transformation){
        return this.withVector(this.getRotation().minus(Rotation2d.fromDegrees(90)), transformation, this.getRotation()); // minus 90 because 0 axis changes to Y
    }
}
