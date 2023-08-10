package frc.robot.diagnostics;

import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Diagnostics {
    private ConcurrentLinkedQueue<DeviceStatus> devices;
    private static Diagnostics instance;
    private boolean shouldRun = true;

    private Diagnostics() {
        devices = new ConcurrentLinkedQueue<DeviceStatus>();
        start();
    }

    public static Diagnostics getInstance() {
        if (instance == null) {
            instance = new Diagnostics();
        }
        return instance;
    }

    public void register(TalonFX talonFX, String name) {
        devices.add(new TalonFXStatus(talonFX, name));
    }

    public void register(CANSparkMax sparkMAX, String name) {
        devices.add(new SparkMaxStatus(sparkMAX, name));
    }

    public void start() {
        if (shouldRun) {
            new Thread(() -> {
                while (shouldRun) {
                    // could use a ScheduledExecutorService if we do not want to check the status constantly 
                    for (DeviceStatus device : devices) {
                        device.checkStatus();
                    }
                }
            }).start();
        }
    }

    public void stopDiagnostics() {
        shouldRun = false;
    }
}
