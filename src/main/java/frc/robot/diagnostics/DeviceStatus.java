package frc.robot.diagnostics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DeviceStatus {
    protected String deviceName;
    protected DeviceStatusEnum deviceStatus;

    public DeviceStatus(String deviceName) {
        this.deviceName = deviceName;
        this.deviceStatus = DeviceStatusEnum.ERROR; // Default status is error because we don't know if the device is working or not until we check
    }

    // Enum to represent device status
    protected enum DeviceStatusEnum {
        WORKING, ERROR
    }

    public abstract void checkStatus();
}
