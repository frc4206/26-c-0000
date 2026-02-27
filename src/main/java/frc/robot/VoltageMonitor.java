package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

import java.util.ArrayDeque;

public class VoltageMonitor {

    private final double batteryResistance;
    private final PowerDistribution pdh = new PowerDistribution();

    // Max history size
    private static final int MAX_SIZE = 500;

    // Rolling mean state (instance-based now)
    private final ArrayDeque<Double> voltageList = new ArrayDeque<>();
    private double runningSum = 0.0;

    public VoltageMonitor(double batteryResistance) {
        this.batteryResistance = batteryResistance;

        // Add initial voltage reading properly
        double initialVoltage = RobotController.getBatteryVoltage();
        voltageList.add(initialVoltage);
        runningSum = initialVoltage;
    }

    // Clears all stored history
    public void clearHistory() {
        voltageList.clear();
        runningSum = 0.0;
    }

    public double getUnloadedVoltage() {

        double measuredVoltage = RobotController.getBatteryVoltage();
        double measuredAmps = pdh.getTotalCurrent();

        // V_unloaded = V_measured + I * R
        double calculatedVoltage =
                measuredVoltage + (measuredAmps * batteryResistance);

        // Add new value
        voltageList.add(calculatedVoltage);
        runningSum += calculatedVoltage;

        // Enforce max size
        if (voltageList.size() > MAX_SIZE) {
            double removed = voltageList.removeFirst();
            runningSum -= removed;
        }

        return runningSum / voltageList.size();
    }
}