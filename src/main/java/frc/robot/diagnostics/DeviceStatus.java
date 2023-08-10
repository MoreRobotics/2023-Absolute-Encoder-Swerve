package frc.robot.diagnostics;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DeviceStatus {
    protected String deviceName;
    protected DeviceStatusEnum deviceStatus;
    protected boolean shouldClearStickyFaults;

    public DeviceStatus(String deviceName) {
        this.deviceName = deviceName;
        this.deviceStatus = DeviceStatusEnum.WORKING; // Default status is working because we assume it is fine until we find somethign wrong
    }

    // Enum to represent device status
    protected enum DeviceStatusEnum {
        WORKING, ERROR
    }

    public abstract void checkStatus();
}
