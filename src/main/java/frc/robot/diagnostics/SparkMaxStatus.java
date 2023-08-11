package frc.robot.diagnostics;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkMaxStatus extends DeviceStatus {
    private CANSparkMax sparkMax;

    public SparkMaxStatus(CANSparkMax sparkMax, String deviceName, String deviceType) {
        super(deviceName, deviceType);
        this.sparkMax = sparkMax;
    }

    @Override
    public void checkStatus() {
        checkBusVoltage();

        // Don't need to the below values for anything, just helpful to see.
        SmartDashboard.putNumber("SparkMAXs/" + deviceName + "/Applied Output", sparkMax.getAppliedOutput());
        SmartDashboard.putNumber("SparkMAXs/" + deviceName + "/Output Current", sparkMax.getOutputCurrent());
    }

    @Override
    protected void clearStickyFaults() {
        sparkMax.clearFaults();
    }

    private void checkBusVoltage() {
        double busVoltage = sparkMax.getBusVoltage();
        updateStatus(busVoltage, busVoltage < 9.0, "/Bus Voltage");
    }
    
}
