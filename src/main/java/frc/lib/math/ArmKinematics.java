//package frc.lib.math;
//
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.util.Units;
//
//public class ArmKinematics {
//  private final double armLength;
//  private final double wristLength;
//
//  public ArmKinematics(double armLength, double wristLength) {
//    this.armLength = armLength;
//    this.wristLength = wristLength;
//  }
//
//  public void setPosition(Translation2d position) {
//    this.position = position;
//  }
//
//  public Translation2d getPosition() {
//    return position;
//  }
//
//  /**
//   * @param wristAngle Wrist angle in radians relative to the ground
//   */
//  public void setWristAngle(double wristAngle) {
//    this.wristAngle = Units.degreesToRadians(wristAngle);
//  }
//
//  public double getWristAngle() {
//    return wristAngle;
//  }
//
//  private Translation2d getWristTranslation() {
//    double x = Math.cos(this.wristAngle) * this.wristLength;
//    double y = Math.sin(this.wristAngle) * this.wristLength;
//
//    return new Translation2d(x, y);
//  }
//
//  /**
//   * @param armAngle   arm angle in radians
//   * @param wristAngle wrist angle in radians
//   * @return Arm position
//   */
//  public Translation2d calculateCurrentPosition(double armAngle, double wristAngle) {
//    double armX = Math.cos(armAngle) * this.armLength;
//    double armY = Math.sin(armAngle) * this.armLength;
//
//    double wristFinalAngle = wristAngle - (Math.PI / 2 - armAngle);
//    double wristX = Math.sin(wristFinalAngle) * this.wristLength;
//    double wristY = Math.cos(wristFinalAngle) * this.wristLength;
//
//    return new Translation2d(armX + wristX, armY + wristY);
//  }
//
//  /**
//   * @param armAngle   Arm angle in radians
//   * @param wristAngle Wrist angle relative to the ground in radians
//   * @return wristAngle relative to the arm
//   */
//  public double calculate(double armAngle, double wristAngle) {
//    return Math.PI - armAngle - wristAngle;
//  }
//}
