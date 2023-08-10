package frc.robot.diagnostics;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFXStatus extends DeviceStatus {
    private TalonFX talonFX;

    public TalonFXStatus(TalonFX talonFX, String deviceName, String deviceType) {
        super(deviceName, deviceType);
        this.talonFX = talonFX;
    }

    @Override
    public void checkStatus() {
        SmartDashboard.putNumber("TalonFXs/" + deviceName + "/Bus Voltage", talonFX.getBusVoltage());
        SmartDashboard.putNumber("TalonFXs/" + deviceName + "/Supply Current", talonFX.getSupplyCurrent());

        checkFaults();
        checkStickyFaults();
        checkLastError();
        clearStickyFaults();
    }

    @Override
    protected void clearStickyFaults() {
        handleErrorCode(talonFX.clearStickyFaults());
    }

    private void checkFaults() {
        Faults faults = new Faults();
        handleErrorCode(talonFX.getFaults(faults));
        updateStatus(parseFaultString(faults), faults.hasAnyFault(), "/Faults");
    }

    private void checkStickyFaults() {
        StickyFaults stickyFaults = new StickyFaults();
        handleErrorCode(talonFX.getStickyFaults(stickyFaults));
        updateStatus(parseStickyFaultString(stickyFaults), stickyFaults.hasAnyFault(), "/Sticky Faults");
    }

    private void checkLastError() {
        ErrorCode errorCode = talonFX.getLastError();
        updateStatus(errorCode.toString(), errorCode != ErrorCode.OK, "/Last Error");
    }

    /* Helper Methods */

    private void handleErrorCode(ErrorCode errorCode) {
        updateStatus(errorCode.toString(), errorCode != ErrorCode.OK, "/Current Error");
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

    // TODO: Move to general util location
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
