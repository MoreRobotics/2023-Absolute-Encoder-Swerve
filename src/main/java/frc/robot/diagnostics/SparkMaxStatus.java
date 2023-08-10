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
        SmartDashboard.putNumber("SparkMAXs/" + deviceName + "/Bus Voltage", sparkMax.getBusVoltage());
        SmartDashboard.putNumber("SparkMAXs/" + deviceName + "/Applied Output", sparkMax.getAppliedOutput());
        SmartDashboard.putNumber("SparkMAXs/" + deviceName + "/Output Current", sparkMax.getOutputCurrent());
    }

    @Override
    protected void clearStickyFaults() {
        sparkMax.clearFaults();
    }
    
}
