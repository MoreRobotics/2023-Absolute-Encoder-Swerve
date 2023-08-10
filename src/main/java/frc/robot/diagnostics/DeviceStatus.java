package frc.robot.diagnostics;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DeviceStatus {
    protected String deviceName;
    protected String deviceType;
    protected DeviceStatusEnum deviceStatus;
    protected boolean shouldClearStickyFaults = false;
    private GenericEntry shouldClearStickyFaultsEntry;

    public DeviceStatus(String deviceName, String deviceType) {
        this.deviceName = deviceName;
        this.deviceType = deviceType;
        this.deviceStatus = DeviceStatusEnum.WORKING; // Default status is working because we assume it is fine until we find something wrong
        shouldClearStickyFaultsEntry = 
            Shuffleboard.getTab("Clear Sticky Faults")
                .add(deviceName, shouldClearStickyFaults)
                .getEntry();
    }

    public void periodic() {
        SmartDashboard.putBoolean("Diagnostics/" + deviceName, deviceStatus == DeviceStatusEnum.WORKING);
        checkIfShouldClearStickyFaults();
        checkStatus();
    }

    protected abstract void checkStatus();

    // Enum to represent device status
    protected enum DeviceStatusEnum {
        WORKING, ERROR
    }

    protected abstract void clearStickyFaults();

    protected void updateStatus(boolean value, boolean isError, String label) {
        deviceStatus = isError ? DeviceStatusEnum.ERROR : DeviceStatusEnum.WORKING;
        SmartDashboard.putBoolean(deviceType + deviceName + label, value);
    }
    
    protected void updateStatus(double value, boolean isError, String label) {
        deviceStatus = isError ? DeviceStatusEnum.ERROR : DeviceStatusEnum.WORKING;
        SmartDashboard.putNumber(deviceType + deviceName + label, value);
    }

    protected void updateStatus(String value, boolean isError, String label) {
        deviceStatus = isError ? DeviceStatusEnum.ERROR : DeviceStatusEnum.WORKING;
        SmartDashboard.putString(deviceType + deviceName + label, value);
    }

    private void checkIfShouldClearStickyFaults() {
        shouldClearStickyFaults = shouldClearStickyFaultsEntry.getBoolean(false);
        if (shouldClearStickyFaults) {
            clearStickyFaults();
        }
    }
}
