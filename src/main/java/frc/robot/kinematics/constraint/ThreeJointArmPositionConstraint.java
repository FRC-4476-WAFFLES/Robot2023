package frc.robot.kinematics.constraint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ThreeJointArmPositionConstraint {
  public Pose2d getClosestPoint(
    Pose2d targetPose
  );

  public boolean isOutsideBounds(
    Pose2d targetPose
  );

  public Pose2d getLimitedPoint(
    Pose2d targetPose
  );
  
  public static interface MathFunction {
    abstract double calculate(Rotation2d rotation);
  }
}
