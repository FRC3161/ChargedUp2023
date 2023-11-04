package frc.lib.logging;

public class SysIdGeneralMechanismLogger extends SysIdLogger {
    private double primaryMotorVoltage = 0.0;

    public double getMotorVoltage() {
        return primaryMotorVoltage;
    }

    public void log(double voltage, double measuredPosition, double measuredVelocity) {
        // set desired voltage
        updateData();
        // if we have room left in the data list
        if (data.size() < dataVectorSize) {
            // add datapoints to list
            double[] dataPacket = new double[] { timestamp, voltage, measuredPosition, measuredVelocity };
            for (double d : dataPacket) {
                data.add(d);
            }
        }
        // update desired motor voltage
        primaryMotorVoltage = motorVoltage;
    }

    public void reset() {
        super.reset();
        primaryMotorVoltage = 0.0;
    }

    @Override
    boolean isWrongMechanism() {
        return !mechanism.equals("Arm") && !mechanism.equals("Elevator") && !mechanism.equals("Simple");
    }
}