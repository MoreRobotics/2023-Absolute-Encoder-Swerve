package frc.robot.diagnostics;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFXStatus extends DeviceStatus {
    private TalonFX talonFX;

    public TalonFXStatus(TalonFX talonFX, String deviceName) {
        super(deviceName);
        this.talonFX = talonFX;
    }

    @Override
    public void checkStatus() {
        SmartDashboard.putBoolean("Diagnostics/" + deviceName, deviceStatus == DeviceStatusEnum.WORKING);
        SmartDashboard.putNumber("TalonFXs/" + deviceName + "/Bus Voltage", talonFX.getBusVoltage());
        SmartDashboard.putNumber("TalonFXs/" + deviceName + "/Supply Current", talonFX.getSupplyCurrent());

        checkFaults();
        checkStickyFaults();
        checkLastError();
    }

    private void checkFaults() {
        Faults faults = new Faults();
        handleErrorCode(talonFX.getFaults(faults));
        if (faults.hasAnyFault()) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Faults", faults.toString());
    }

    private void checkStickyFaults() {
        StickyFaults stickyFaults = new StickyFaults();
        handleErrorCode(talonFX.getStickyFaults(stickyFaults));
        if (stickyFaults.hasAnyFault()) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Sticky Faults", stickyFaults.toString());
    }

    private void checkLastError() {
        ErrorCode errorCode = talonFX.getLastError();
        if (errorCode != ErrorCode.OK) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Last Error", errorCode.toString());
    }

    private void handleErrorCode(ErrorCode errorCode) {
        if (errorCode != ErrorCode.OK) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Current Error", errorCode.toString());
    }
}
