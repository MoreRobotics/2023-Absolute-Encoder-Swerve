package frc.robot.diagnostics;

import java.util.ArrayList;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFXStatus extends DeviceStatus {
    private TalonFX talonFX;

    public TalonFXStatus(TalonFX talonFX, String deviceName) {
        super(deviceName);
        this.talonFX = talonFX;
        shouldClearStickyFaults = false; // For now this does nothing as I can't figure out how to activate this from Shuffleboard
    }

    @Override
    public void checkStatus() {
        SmartDashboard.putBoolean("Diagnostics/" + deviceName, deviceStatus == DeviceStatusEnum.WORKING);
        SmartDashboard.putNumber("TalonFXs/" + deviceName + "/Bus Voltage", talonFX.getBusVoltage());
        SmartDashboard.putNumber("TalonFXs/" + deviceName + "/Supply Current", talonFX.getSupplyCurrent());
        SmartDashboard.putBoolean("TalonFXs/" + deviceName + "/Clear Sticky Faults", shouldClearStickyFaults);

        checkFaults();
        checkStickyFaults();
        checkLastError();
        clearStickyFaults();
    }

    private void checkFaults() {
        Faults faults = new Faults();
        handleErrorCode(talonFX.getFaults(faults));
        if (faults.hasAnyFault()) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Faults", parseFaultString(faults));
    }

    private void checkStickyFaults() {
        StickyFaults stickyFaults = new StickyFaults();
        handleErrorCode(talonFX.getStickyFaults(stickyFaults));
        if (stickyFaults.hasAnyFault()) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Sticky Faults", parseStickyFaultString(stickyFaults));
    }

    private void checkLastError() {
        ErrorCode errorCode = talonFX.getLastError();
        if (errorCode != ErrorCode.OK) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Last Error", errorCode.toString());
    }

    private void clearStickyFaults() {
        if (shouldClearStickyFaults) {
            handleErrorCode(talonFX.clearStickyFaults());
        }
    }

    /* Helper Methods */

    private void handleErrorCode(ErrorCode errorCode) {
        if (errorCode != ErrorCode.OK) {
            deviceStatus = DeviceStatusEnum.ERROR;
        }
        SmartDashboard.putString("TalonFXs/" + deviceName + "/Current Error", errorCode.toString());
    }

    private String parseFaultString(Faults faults) {
        String[] faultString = faults.toString().split(" ");
        ArrayList<Integer> faultBitfield = intToBinaryList(faults.toBitfield());

        ArrayList<String> faultsArray = new ArrayList<>();
        for (int i=0; i < faultBitfield.size(); i++) {
            try {
                if (faultBitfield.get(i) == 1) {
                    faultsArray.add(faultString[i + 1]);
                }
            } catch (Exception e) {
                System.out.println("Probably overran the faultString array in parseFaultString()");
            }
        }

        return faultsArray.toString();
    }

    private String parseStickyFaultString(StickyFaults faults) {
        String[] faultString = faults.toString().split(" ");
        ArrayList<Integer> faultBitfield = intToBinaryList(faults.toBitfield());

        ArrayList<String> faultsArray = new ArrayList<>();
        for (int i=0; i < faultBitfield.size(); i++) {
            try {
                if (faultBitfield.get(i) == 1) {
                    faultsArray.add(faultString[i + 1]);
                }
            } catch (Exception e) {
                System.out.println("Probably overran the faultString array in parseStickyFaultString()");
            }
        }

        return faultsArray.toString();
    }

    private ArrayList<Integer> intToBinaryList(int number) {
        ArrayList<Integer> binaryList = new ArrayList<>();

        while (number > 0) {
            binaryList.add(number % 2);
            number /= 2;
        }

        if (binaryList.isEmpty()) {
            binaryList.add(0);
        }

        return binaryList;
    }
}
