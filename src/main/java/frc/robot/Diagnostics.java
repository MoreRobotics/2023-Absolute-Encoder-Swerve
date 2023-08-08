package frc.robot;

import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Diagnostics {
    private ConcurrentHashMap<String, TalonFX> talonFXs; 
    private ConcurrentHashMap<String, CANSparkMax> sparkMAXs;
    private static Diagnostics instance;
    private boolean shouldRun = true;

    private Diagnostics() {
        talonFXs = new ConcurrentHashMap<String, TalonFX>();
        sparkMAXs = new ConcurrentHashMap<String, CANSparkMax>();
        start();
    }

    public static Diagnostics getInstance() {
        if (instance == null) {
            instance = new Diagnostics();
        }
        return instance;
    }

    public void register(TalonFX talonFX, String name) {
        talonFXs.put(name, talonFX);
    }

    public void register(CANSparkMax sparkMAX, String name) {
        sparkMAXs.put(name, sparkMAX);
    }

    public void start() {
        if (shouldRun) {
            new Thread(() -> {
                while (shouldRun) {
                    for (Entry<String, TalonFX> entry : talonFXs.entrySet()) {
                        runTalonFXDiagnostics(entry.getValue(), entry.getKey());
                    }
                    for (Entry<String, CANSparkMax> entry : sparkMAXs.entrySet()) {
                        runSparkMAXDiagnostics(entry.getValue(), entry.getKey());
                    }
                }
            }).start();
        }
    }

    public void stopDiagnostics() {
        shouldRun = false;
    }

    private void runTalonFXDiagnostics(TalonFX talonFX, String name) {
        SmartDashboard.putNumber("TalonFXs/" + name + "/Bus Voltage", talonFX.getBusVoltage());
        SmartDashboard.putNumber("TalonFXs/" + name + "/Supply Current", talonFX.getSupplyCurrent());
    }

    private void runSparkMAXDiagnostics(CANSparkMax sparkMAX, String name) {
        SmartDashboard.putNumber("SparkMAXs/" + name + "/Bus Voltage", sparkMAX.getBusVoltage());
        SmartDashboard.putNumber("SparkMAXs/" + name + "/Applied Output", sparkMAX.getAppliedOutput());
        SmartDashboard.putNumber("SparkMAXs/" + name + "/Output Current", sparkMAX.getOutputCurrent());
    }
}
