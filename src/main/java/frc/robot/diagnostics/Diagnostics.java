package frc.robot.diagnostics;

import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Diagnostics {
    private ConcurrentLinkedQueue<DeviceStatus> devices;
    private static Diagnostics instance;
    private ScheduledExecutorService executorService;

    private Diagnostics() {
        devices = new ConcurrentLinkedQueue<DeviceStatus>();
        executorService = Executors.newScheduledThreadPool(1);
        start();
    }

    public static Diagnostics getInstance() {
        if (instance == null) {
            instance = new Diagnostics();
        }
        return instance;
    }

    public void register(TalonFX talonFX, String name) {
        devices.add(new TalonFXStatus(talonFX, name, "TalonFXs/"));
    }

    public void register(CANSparkMax sparkMAX, String name) {
        devices.add(new SparkMaxStatus(sparkMAX, name, "SparkMAXs/"));
    }

    public void start() {
        executorService.scheduleWithFixedDelay(() -> {
            for (DeviceStatus device : devices) {
                device.periodic();
                periodic();
            }
        }, 0, 1, TimeUnit.SECONDS);
    }

    public void stopDiagnostics() {
        executorService.shutdown();
    }

    private void periodic() {
        CANStatus canStatus = RobotController.getCANStatus();
        SmartDashboard.putNumber("Diagnostics/CAN Utilization", canStatus.percentBusUtilization);
    }
}
