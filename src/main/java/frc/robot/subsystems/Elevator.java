// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMode;
import frc.robot.RobotMode.ModeOptions;
import frc.robot.RobotMode.StateOptions;


public class Elevator extends SubsystemBase {

    public static final int FORWARD_ELEVATOR_LIMIT = 26;
    public static final float REVERSE_ELEVATOR_LIMIT = (float) -0.5;
    public static final int ELEVATOR_MOTOR_ID = 16;
    public static final double ELEVATOR_GEAR_RATIO = 9.0;
    public static final double ELEVATOR_SPROCKET_DIAMETER = 1.751;
    public static final double ELEVATOR_ROTATIONS_TO_IN = 1.0/ELEVATOR_GEAR_RATIO * ELEVATOR_SPROCKET_DIAMETER * Math.PI;
    public static final double MANUAL_ELEVATOR_SPEED = 0.50;
    public static final double ELEVATOR_P = 1.0;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.0;

    public static final double ELEVATOR_CONE_HIGH_LEVEL = 26;
    public static final double ELEVATOR_CONE_MID_LEVEL = 8.5;
    public static final double ELEVATOR_CONE_LOW_LEVEL = REVERSE_ELEVATOR_LIMIT;
    public static final double ELEVATOR_CONE_SAFE_LEVEL = 10.0;
    public static final double ELEVATOR_CONE_STOW_LEVEL = 6.0;
    public static final double ELEVATOR_CONE_SINGLE_POSITION = 4;
    public static final double ELEVATOR_CONE_DOUBLE_POSITION = 4;

    public static final double ELEVATOR_CUBE_HIGH_LEVEL = 26;
    public static final double ELEVATOR_CUBE_MID_LEVEL = 8.5;
    public static final double ELEVATOR_CUBE_LOW_LEVEL = REVERSE_ELEVATOR_LIMIT;
    public static final double ELEVATOR_CUBE_SAFE_LEVEL = 10.0;
    public static final double ELEVATOR_CUBE_STOW_LEVEL = 6.0;
    public static final double ELEVATOR_CUBE_SINGLE_POSITION = 4;
    public static final double ELEVATOR_CUBE_DOUBLE_POSITION = 4;

    public static final double ELEVATOR_DEFUALT_HIGH_LEVEL = 26;
    public static final double ELEVATOR_DEFUALT_MID_LEVEL = 8.5;
    public static final double ELEVATOR_DEFUALT_LOW_LEVEL = REVERSE_ELEVATOR_LIMIT;
    public static final double ELEVATOR_DEFUALT_SAFE_LEVEL = 10.0;
    public static final double ELEVATOR_DEFUALT_STOW_LEVEL = 6.0;
    public static final double ELEVATOR_DEFUALT_SINGLE_POSITION = 4;
    public static final double ELEVATOR_DEFUALT_DOUBLE_POSITION = 4;

    public static final double ELEVATOR_TOLERANCE = 20;
  
    public RelativeEncoder elevatorEncoder;
    private CANSparkMax elevatorMotor;
    private ProfiledPIDController controller;
    private ElevatorFeedforward ff;
    private double targetElevatorPosition;

    /* Logging */
    private DataLog logger;

    // /* Elevator Motor */
    private DoubleLogEntry elevatorMotorTemperature;
    private DoubleLogEntry elevatorMotorAppliedOutput;
    private DoubleLogEntry elevatorMotorOutputCurrent;
    private IntegerLogEntry elevatorMotorFaults;

    // /* Elevator Motor Internal Encoder  */
    // private DoubleLogEntry elevatorEncoderPosition;
    // private DoubleLogEntry elevatorEncoderVelocity;

    public Elevator() {
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        //TODO: Set Current Limiters
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, FORWARD_ELEVATOR_LIMIT);
        elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, REVERSE_ELEVATOR_LIMIT);
        elevatorEncoder = elevatorMotor.getEncoder(); 
        elevatorMotor.setIdleMode(IdleMode.kBrake);

        /* Current Limits */
        elevatorMotor.setSmartCurrentLimit(40);

        /*Rate of Speed (Based on 930 Code) */

        // double p = SmartDashboard.getNumber("p", 0);
        this.controller = new ProfiledPIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, new Constraints(80, 1000));
        this.controller.setTolerance(100, 100);
        // TODO: Recalculate these constants
        //this.ff = new ElevatorFeedforward(0, 0.16, 0.18, 0.02);

        elevatorEncoder.setPositionConversionFactor(ELEVATOR_ROTATIONS_TO_IN);
        elevatorEncoder.setVelocityConversionFactor(ELEVATOR_ROTATIONS_TO_IN);

        // Create logger object 
        logger = DataLogManager.getLog();

        // // Create log entries for elevator motor
        elevatorMotorTemperature = new DoubleLogEntry(logger, "elevatorMotor/temperature");
        elevatorMotorAppliedOutput = new DoubleLogEntry(logger, "elevatorMotor/appliedOutput");
        elevatorMotorOutputCurrent = new DoubleLogEntry(logger, "elevatorMotor/outputCurrent");
        elevatorMotorFaults = new IntegerLogEntry(logger, "elevatorMotor/faults");

        // // Create log entries for elevator encoder
        // elevatorEncoderPosition = new DoubleLogEntry(logger, "elevatorEncoder/position");
        // elevatorEncoderVelocity = new DoubleLogEntry(logger, "elevatorEncoder/velocity");
    }
    
    
    /* Sets the Target Elevator Position in inches.*/
    public void SetTargetElevatorPosition(){
        
        ModeOptions mode = RobotMode.mode;
        StateOptions state = RobotMode.state;

        if (mode == RobotMode.ModeOptions.CUBE) {

            if (state == RobotMode.StateOptions.LOW) {
                targetElevatorPosition = ELEVATOR_CUBE_LOW_LEVEL;
            } else if (state == RobotMode.StateOptions.MID) {
                targetElevatorPosition = ELEVATOR_CUBE_MID_LEVEL;
            } else if (state == RobotMode.StateOptions.HIGH) {
                targetElevatorPosition = ELEVATOR_CUBE_HIGH_LEVEL;
            } else if (state == RobotMode.StateOptions.SINGLE) {
                targetElevatorPosition = ELEVATOR_CUBE_SINGLE_POSITION;
            } else if (state == RobotMode.StateOptions.DOUBLE) {
                targetElevatorPosition = ELEVATOR_CUBE_DOUBLE_POSITION;
            } else {
                targetElevatorPosition = ELEVATOR_CUBE_STOW_LEVEL;
            }

        } else {

            if (state == RobotMode.StateOptions.LOW) {
                targetElevatorPosition = ELEVATOR_CONE_LOW_LEVEL;
            } else if (state == RobotMode.StateOptions.MID) {
                targetElevatorPosition = ELEVATOR_CONE_MID_LEVEL;
            } else if (state == RobotMode.StateOptions.HIGH) {
                targetElevatorPosition = ELEVATOR_CONE_HIGH_LEVEL;
            } else if (state == RobotMode.StateOptions.SINGLE) {
                targetElevatorPosition = ELEVATOR_CONE_SINGLE_POSITION;
            } else if (state == RobotMode.StateOptions.DOUBLE) {
                targetElevatorPosition = ELEVATOR_CONE_DOUBLE_POSITION;
            } else {
                targetElevatorPosition = ELEVATOR_CONE_STOW_LEVEL;
            }
        }


    }

    /* Sets the Target Elevator Position in inches.*/
    public double getTargetElevatorPosition(){
        return targetElevatorPosition;
    }

    public void extend() {

        targetElevatorPosition = targetElevatorPosition + MANUAL_ELEVATOR_SPEED;

        if (targetElevatorPosition >= FORWARD_ELEVATOR_LIMIT) {
            targetElevatorPosition = FORWARD_ELEVATOR_LIMIT;
        }

    }

    public void retract() {

        targetElevatorPosition = targetElevatorPosition - MANUAL_ELEVATOR_SPEED;

        if (targetElevatorPosition <= REVERSE_ELEVATOR_LIMIT) {
            targetElevatorPosition = REVERSE_ELEVATOR_LIMIT;
        }
        
    }

    public void stop() {
        elevatorMotor.set(0);
    } 
    
    public double getEncoderPosition() {
        return elevatorEncoder.getPosition();
    }

    private double motorRotationsToInches(double rotations) {
        return rotations * ELEVATOR_ROTATIONS_TO_IN;
    }

    private double inchesToMotorRotations(double inches) {
        return inches / ELEVATOR_ROTATIONS_TO_IN;
    }

    public boolean atPosition() {

        double error = Math.abs(elevatorEncoder.getPosition() - targetElevatorPosition);

        if (ELEVATOR_TOLERANCE >= error) {
            return true;

        } else {
            return false;
        }
        
    }

    public Boolean isHigh(){
        return getTargetElevatorPosition() == ELEVATOR_DEFUALT_HIGH_LEVEL;
    }
    
    @Override
    public void periodic() {
        if (DriverStation.isEnabled()){
            // This method will be called once per scheduler run
            // TODO: Test that .getPosition() gives us the elevator position in inches
            SetTargetElevatorPosition();
            double voltage = controller.calculate(elevatorEncoder.getPosition(), targetElevatorPosition);
            // double feedforward = ff.calculate(/*double */elevatorEncoder.getVelocity());
            MathUtil.clamp(voltage, -12, 12);

            elevatorMotor.setVoltage(voltage);

            
            SmartDashboard.putNumber("ELEVATOR PID VOLTAGE", voltage);
        }

        SmartDashboard.putNumber("ELEVATOR TARGET POSITION", targetElevatorPosition);
        SmartDashboard.putNumber("Elevator Encoder Value: ", getEncoderPosition());
        
        logData();
    }


    /*
    public Command SetElevatorPosition (double inches){
        return new InstantCommand(() -> setTargetElevatorPosition(inches), this);
    }

    public Command ElevatorAtPosition(){
        return Commands.waitUntil(() -> atPosition());
    }
    */

    private void logData() {
        /* Elevator Motor */
        elevatorMotorTemperature.append(elevatorMotor.getMotorTemperature());
        elevatorMotorAppliedOutput.append(elevatorMotor.getAppliedOutput());
       // elevatorMotorBusVoltage.append(elevatorMotor.getBusVoltage());
        elevatorMotorOutputCurrent.append(elevatorMotor.getOutputCurrent());
      //  elevatorMotorClosedLoopRampRate.append(elevatorMotor.getClosedLoopRampRate());
    //    elevatorMotorOpenLoopRampRate.append(elevatorMotor.getOpenLoopRampRate());
        elevatorMotorFaults.append(elevatorMotor.getFaults());
    //    elevatorMotorIdleMode.append(elevatorMotor.getIdleMode().toString());
    //    elevatorMotorInverted.append(elevatorMotor.getInverted());
     //   elevatorMotorLastError.append(elevatorMotor.getLastError().toString());

        // /* Elevator Encoder */
        // elevatorEncoderPosition.append(getEncoderPosition());
        // elevatorEncoderVelocity.append(elevatorEncoder.getVelocity());
      }
}

