package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurnToNearest90 {
  private final PIDController turnController = new PIDController(-4.0, 0, -0.2);
  private double targetHeading = 0;

  public TurnToNearest90() {}

  public void setTargetHeading(double targetHeading) {
    this.targetHeading = targetHeading;
  }

  public double getTargetHeading() {
    return targetHeading;
  }

  public double calculate(double currentHeading) {
    currentHeading = Math.round(currentHeading * 5) / 5; // Ryan told me to do this to round a number to half a decimal (his words)

    if (currentHeading < -135) {
      targetHeading = -180;
    } else if (currentHeading < -45) {
      targetHeading = -90;
    } else if (currentHeading < 45) {
      targetHeading = 0;
    } else if (currentHeading < 135) {
      targetHeading = 90;
    } else {
      targetHeading = 180;
    }

    //targetHeading += 45; // IDK why this is needed, java bad

    double optimizedTargetAngle = Math.toRadians(currentHeading) + Rotation2d.fromDegrees(targetHeading).minus(Rotation2d.fromDegrees(currentHeading)).getRadians();
    turnController.setSetpoint(optimizedTargetAngle);
    return turnController.calculate(Math.toRadians(currentHeading));
  }
}
