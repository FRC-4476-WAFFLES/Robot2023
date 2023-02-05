package frc.robot.kinematics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.kinematics.constraint.ThreeJointArmPositionConstraint;

public class ThreeJointArmKinematics {
  private final double a1;
  private final double a2;
  private final double a3;

  private final List<ThreeJointArmPositionConstraint> constraints = new ArrayList<>();

  public ThreeJointArmKinematics(double a1, double a2, double a3) {
    if (a1 <= 0 || a2 <= 0) {
      throw new IllegalArgumentException("A1 and A2 must both be greater than 0!");
    } 

    if (a3 < 0) {
      throw new IllegalArgumentException("A3 cannot be less than 0!");
    }

    this.a1 = a1;
    this.a2 = a2;
    this.a3 = a3;
  }

  public ThreeJointArmState toArmState(Pose2d targetPose) {
    Translation2d wristTarget = new Translation2d(
      targetPose.getX() - a3 * Math.cos(targetPose.getRotation().getRadians()), 
      targetPose.getY() - a3 * Math.sin(targetPose.getRotation().getRadians())
    );

    Rotation2d q2Relative = inverseKinematicsElbow(wristTarget);
    Rotation2d q1 = inverseKinematicsShoulder(wristTarget, q2Relative);

    Rotation2d q2 = q2Relative.plus(q1);

    return new ThreeJointArmState(q1, q2, targetPose.getRotation());
  }

  public Pose2d toPose2d(ThreeJointArmState armState) {
    Translation2d p1 = new Translation2d(a1, armState.q1);
    Translation2d p2 = new Translation2d(a2, armState.q2);
    Translation2d p3 = new Translation2d(a3, armState.q3);
    
    return new Pose2d(p1.plus(p2).plus(p3), armState.q3);
  }

  public void addConstraint(ThreeJointArmPositionConstraint constraint) {
    constraints.add(constraint);
  }

  public Pose2d limitTargetPosition(Pose2d targetPose) {
    Pose2d adjustedPose = targetPose;
    for (ThreeJointArmPositionConstraint constraint : constraints) {
      adjustedPose = constraint.getClosestPoint(adjustedPose);
    }

    return adjustedPose;
  }

  /**
   * Inverse kinematics to determine the angle of the elbow given a target pos for the wrist. Calculation source is Robot Academy: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
   * @param targetPos The target position of the wrist
   * @return The angle of the elbow
   */
  private Rotation2d inverseKinematicsElbow(Translation2d targetPos) {
    return targetPos.getX() > 0 
      ? Rotation2d.fromRadians(-Math.acos((Math.pow(targetPos.getX(), 2) + Math.pow(targetPos.getY(), 2) - Math.pow(a1, 2) - Math.pow(a2, 2)) / (2 * a1 *  a2))) 
      : Rotation2d.fromRadians(Math.acos((Math.pow(targetPos.getX(), 2) + Math.pow(targetPos.getY(), 2) - Math.pow(a1, 2) - Math.pow(a2, 2)) / (2 * a1 *  a2)));
  }

  /**
   * Inverse kinematics to determine the angle of the shoulder given a target pos for the wrist and the calculated elbow angle. Calculation source is Robot Academy: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
   * @param targetPos The target position of the wrist
   * @param elbowAngle The elbow angle as calculated by {@link #inverseKinematicsElbow(Translation2d)}
   * @return The angle of the shoulder
   */
  private Rotation2d inverseKinematicsShoulder(Translation2d targetPos, Rotation2d elbowAngle) {
    return targetPos.getX() > 0 
      ? Rotation2d.fromRadians(Math.atan(targetPos.getY() / Math.abs(targetPos.getX())) - Math.atan((a2 * Math.sin(elbowAngle.getRadians())) / (a1 + a2 * Math.cos(elbowAngle.getRadians()))))
      : Rotation2d.fromRadians(Math.PI - Math.atan(targetPos.getY() / Math.abs(targetPos.getX())) - Math.atan((a2 * Math.sin(elbowAngle.getRadians())) / (a1 + a2 * Math.cos(elbowAngle.getRadians()))));
  }
}
