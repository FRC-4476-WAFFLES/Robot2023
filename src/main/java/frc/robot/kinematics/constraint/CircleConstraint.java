package frc.robot.kinematics.constraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class CircleConstraint implements ThreeJointArmPositionConstraint {
  private final MathFunction h;
  private final MathFunction k;
  private final double r;
  private final boolean isExternalBound;

  public CircleConstraint(double h, double k, double r, boolean isOutsideBound) {
    this((rotation) -> h, (rotation) -> k, r, isOutsideBound);
  }

  public CircleConstraint(MathFunction h, MathFunction k, double r, boolean isOutsideBound) {
    this.h = h;
    this.k = k;
    this.r = r;
    this.isExternalBound = isOutsideBound;
  }

  @Override
  public Pose2d getClosestPoint(Pose2d targetPose) {
    Rotation2d rotation = targetPose.getRotation();

    double vX = targetPose.getX() - h.calculate(rotation);
    double vY = targetPose.getY() - k.calculate(rotation);
    double magV = Math.sqrt(vX*vX + vY*vY);
    double aX = h.calculate(rotation) + vX / magV * r;
    double aY = k.calculate(rotation) + vY / magV * r;

    return new Pose2d(aX, aY, rotation);
  }

  @Override
  public boolean isOutsideBounds(Pose2d targetPose) {
    Rotation2d rotation = targetPose.getRotation();

    double vX = targetPose.getX() - h.calculate(rotation);
    double vY = targetPose.getY() - k.calculate(rotation);
    double magV = Math.sqrt(vX*vX + vY*vY);

    return isExternalBound ? magV < r : magV > r;
  }

  @Override
  public Pose2d getLimitedPoint(Pose2d targetPose) {
    Rotation2d rotation = targetPose.getRotation();

    double vX = targetPose.getX() - h.calculate(rotation);
    double vY = targetPose.getY() - k.calculate(rotation);
    double magV = Math.sqrt(vX*vX + vY*vY);
    double aX = h.calculate(rotation) + vX / magV * r;
    double aY = k.calculate(rotation) + vY / magV * r;

    // Return the target pose if it satisfies the bounding condition, otherwise return the closest point. 
    if (isExternalBound) {
      return magV < r ? targetPose : new Pose2d(aX, aY, rotation);
    } else {
      return magV > r ? targetPose : new Pose2d(aX, aY, rotation);
    }
  }
}
