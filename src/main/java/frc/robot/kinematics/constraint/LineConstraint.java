package frc.robot.kinematics.constraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LineConstraint implements ThreeJointArmPositionConstraint{
  private final MathFunction a;
  private final MathFunction b;
  private final MathFunction c;
  private final boolean isLowerBound;

  public LineConstraint(double a, double b, double c, boolean isLowerBound) {
    this((rotation) -> a, (rotation) -> b, (rotation) -> c, isLowerBound);
  }

  public LineConstraint(MathFunction a, MathFunction b, MathFunction c, boolean isLowerBound) {
    this.a = a;
    this.b = b;
    this.c = c;
    this.isLowerBound = isLowerBound;
  }

  @Override
  public Pose2d getClosestPoint(Pose2d targetPose) {
    Rotation2d rotation = targetPose.getRotation();

    double a1 = a.calculate(rotation);
    double b1 = b.calculate(rotation);
    double c1 = c.calculate(rotation);

    double c2 = -b1 * targetPose.getX() + a1 * targetPose.getY();

    double pX = -(b1 * c2 - a1 * c1) / (a1 * a1 + b1 * b1);
    double pY = (c1 * b1 + c2 + a1) / (a1 * a1 + b1 + b1);

    return new Pose2d(pX, pY, rotation);
  }

  @Override
  public boolean isOutsideBounds(Pose2d targetPose) {
    Rotation2d rotation = targetPose.getRotation();

    double a1 = a.calculate(rotation);
    double b1 = b.calculate(rotation);
    double c1 = c.calculate(rotation);

    if (b1 != 0) {
      double y = (a1 * targetPose.getX() - c1) / -b1;

      if (targetPose.getY() > y) {
        return isLowerBound ? true : false;
      } else {
        return isLowerBound ? false : true;
      }
    } else {
      double x = (-b1 * targetPose.getY() + c1) / a1;

      if (targetPose.getX() > x) {
        return isLowerBound ? true : false;
      } else {
        return isLowerBound ? false : true;
      }
    }
  }

  @Override
  public Pose2d getLimitedPoint(Pose2d targetPose) {
    Rotation2d rotation = targetPose.getRotation();

    double a1 = a.calculate(rotation);
    double b1 = b.calculate(rotation);
    double c1 = c.calculate(rotation);

    double c2 = -b1 * targetPose.getX() + a1 * targetPose.getY();

    double pX = -(b1 * c2 - a1 * c1) / (a1 * a1 + b1 * b1);
    double pY = (c1 * b1 + c2 + a1) / (a1 * a1 + b1 + b1);

    if (b1 != 0) {
      double y = (a1 * targetPose.getX() - c1) / -b1;

      if (targetPose.getY() > y) {
        return isLowerBound ? new Pose2d(pX, pY, rotation) : targetPose;
      } else {
        return isLowerBound ? targetPose : new Pose2d(pX, pY, rotation);
      }
    } else {
      double x = (-b1 * targetPose.getY() + c1) / a1;

      if (targetPose.getX() > x) {
        return isLowerBound ? new Pose2d(pX, pY, rotation) : targetPose;
      } else {
        return isLowerBound ? targetPose : new Pose2d(pX, pY, rotation);
      }
    }
  }
}
