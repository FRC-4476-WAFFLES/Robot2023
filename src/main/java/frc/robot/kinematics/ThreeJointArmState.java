package frc.robot.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;

public class ThreeJointArmState {
  public Rotation2d q1 = Rotation2d.fromDegrees(0);
  public Rotation2d q2 = Rotation2d.fromDegrees(0);
  public Rotation2d q3 = Rotation2d.fromDegrees(0);

  public ThreeJointArmState() {}

  public ThreeJointArmState(Rotation2d q1, Rotation2d q2, Rotation2d q3) {
    this.q1 = q1;
    this.q2 = q2;
    this.q3 = q3;
  }

  @Override
  public String toString() {
    return "Three Joint Arm State(Q1: " + q1.toString() + ", Q2: " + q2.toString() + ", Q3; " + q3.toString() + ")";
  }
}
