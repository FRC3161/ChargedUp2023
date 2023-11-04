// Source: https://github.com/FRC6498/2023ChargedUp/blob/sysid/src/main/java/frc/robot/SysId/Logging

package frc.lib.logging;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class SysIdLogger {
    String mechanism = "";
    String testType = "";
    String test = "";
    boolean rotate = false;
    double voltageCommand = 0.0;
    double motorVoltage = 0.0;
    double startTime = 0.0;
    double timestamp = 0.0;
    double ackNum = 0;
    final int dataVectorSize = 36000;
    ArrayList<Double> data;

    public SysIdLogger() {
        data = new ArrayList<>(dataVectorSize);
    }

    public void initLogging() {
        mechanism = SmartDashboard.getString("SysIdTest", "");
        if (isWrongMechanism()) {
            SmartDashboard.putBoolean("SysIdWrongMech", true);
        }
        testType = SmartDashboard.getString("SysIdTestType", "");
        rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        startTime = Timer.getFPGATimestamp();
        data.clear();
        SmartDashboard.putString("SysIdTelemetry", "");
        ackNum = SmartDashboard.getNumber("SysIdAckNumber", 0);
    }

    public double measureVoltage(List<CANSparkMax> controllers) {
        double sum = 0.0;
        for (int i = 0; i < controllers.size(); ++i) {
            CANSparkMax neo = controllers.get(i);
            sum += neo.getBusVoltage() * neo.getAppliedOutput();
        }
        return sum / controllers.size();
    }

    public void sendData() {
        System.out.println("Collected: " + data.size() + " data points.\n");

        SmartDashboard.putBoolean("SysIdOverflow", data.size() >= dataVectorSize);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < data.size(); ++i) {
            sb.append(data.get(i));
            if (i < data.size() - 1) {
                sb.append(",");
            }
        }

        String type = testType.equals("Dynamic") ? "fast" : "slow";
        String direction = voltageCommand > 0 ? "forward" : "backward";
        String test = new StringBuilder().append(type).append("-").append(direction).toString();

        SmartDashboard.putString("SysIdTelemetry",
                new StringBuilder().append(test).append(";").append(sb.toString()).toString());
        SmartDashboard.putNumber("SysIdAckNumber", ++ackNum);
        reset();
    }

    public void clearWhenRecieved() {
        if (SmartDashboard.getNumber("SysIdAckNumber", 0.0) > ackNum) {
            SmartDashboard.putString("SysIdTelemetry", "");
            ackNum = SmartDashboard.getNumber("SysIdAckNumber", 0.0);
        }
    }

    public void updateThreadPriority() {
        if (!Robot.isSimulation()) {
            if (!Notifier.setHALThreadPriority(true, 40) || Threads.setCurrentThreadPriority(true, 15)) {
                throw new RuntimeException("Setting RT priority failed");
            }
        }
    }

    public void updateData() {
        timestamp = Timer.getFPGATimestamp();
        // if the mechanism is supported
        if (!isWrongMechanism()) {
            // if ramp test
            if (testType.equals("Quasistatic")) {
                // voltage = ramp rate * test elapsed time
                motorVoltage = voltageCommand * (timestamp - startTime);
            } else if (testType.equals("Dynamic")) { // if steady state test
                // voltage = steady state voltage
                motorVoltage = voltageCommand;
            } else {
                motorVoltage = 0.0;
            }
        } else {
            motorVoltage = 0.0;
        }
    }

    public void reset() {
        motorVoltage = 0.0;
        timestamp = 0.0;
        startTime = 0.0;
        data.clear();
    }

    public void setMotorControllers(double volts, List<CANSparkMax> motors) {
        for (CANSparkMax motor : motors) {
            motor.setVoltage(volts);
        }
    }

    boolean isWrongMechanism() {
        return false;
    }
}
