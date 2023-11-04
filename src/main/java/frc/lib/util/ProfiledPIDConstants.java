package frc.lib.util;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProfiledPIDConstants {
  public double p, i, d;
  public double maxVelocity, maxAcceleration;
  public double tolerance = 0;
  private String subscript;

  public ProfiledPIDConstants(double p, double i, double d, double maxVelocity, double maxAcceleration) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
  }

  public ProfiledPIDConstants(double p, double i, double d, double maxVelocity, double maxAcceleration,
      double tolerance) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.tolerance = tolerance;
  }

  public ProfiledPIDController getController() {
    ProfiledPIDController controller = new ProfiledPIDController(this.p, this.i, this.d,
        new TrapezoidProfile.Constraints(this.maxVelocity, this.maxAcceleration));
    if (tolerance != 0) {
      controller.setTolerance(tolerance);
    }

    return controller;
  }

  public void sendDashboard(String subscript) {
    this.subscript = subscript;
    SmartDashboard.putNumber(subscript + " P", this.p);
    SmartDashboard.putNumber(subscript + " I", this.i);
    SmartDashboard.putNumber(subscript + " D", this.d);
    SmartDashboard.putNumber(subscript + " Max V", this.maxVelocity);
    SmartDashboard.putNumber(subscript + " Max A", this.maxAcceleration);
  }

  public ProfiledPIDController retrieveDashboard(ProfiledPIDController controller) {
    double p = SmartDashboard.getNumber(this.subscript + " P", 0.0);
    double i = SmartDashboard.getNumber(this.subscript + " I", 0.0);
    double d = SmartDashboard.getNumber(this.subscript + " D", 0.0);
    double maxVelocity = SmartDashboard.getNumber(this.subscript + " Max V", this.maxVelocity);
    double maxAcceleration = SmartDashboard.getNumber(this.subscript + " Max A", this.maxAcceleration);

    if (this.p != p) {
      controller.setP(p);
      this.p = p;
    }
    if (this.i != i) {
      controller.setI(i);
      this.i = i;
    }
    if (this.d != d) {
      controller.setD(d);
      this.d = d;
    }

    if (this.maxVelocity != maxVelocity) {
      controller.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, this.maxAcceleration));
      this.maxVelocity = maxVelocity;
    }

    if (this.maxAcceleration != maxAcceleration) {
      controller.setConstraints(new TrapezoidProfile.Constraints(this.maxVelocity, maxAcceleration));
      this.maxAcceleration = maxAcceleration;
    }

    return controller;
  }
}
