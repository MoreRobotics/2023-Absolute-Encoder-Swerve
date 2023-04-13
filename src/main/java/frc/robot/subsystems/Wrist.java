// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.Logger;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMode.ModeOptions;
import frc.robot.RobotMode.StateOptions;
import frc.robot.RobotMode;

public class Wrist extends SubsystemBase {

  public static final int WRIST_MOTOR_ID = 20;
  public static final int WRIST_ENCODER_ID = 1;
  public static final double WRIST_FORWARD_LIMIT = 40;
  public static final double WRIST_REVERSE_LIMIT = -90;
  public static final double WRIST_GEAR_RATIO = 87.27;
  public static final double WRIST_MOTOR_ROT_TO_DEG = 360.0 / WRIST_GEAR_RATIO;
  public static final double MANUAL_WRIST_SPEED = 1.0;
  public static final double WRIST_GROUND_POSITION = -95; //set value
  // positions
  public static final double WRIST_CONE_STOW_POSITION = 68.82;
  public static final double WRIST_CONE_LOW_POSITION = -25.8;
  public static final double WRIST_CONE_MID_POSITION = 0;
  public static final double WRIST_CONE_HIGH_POSITION = 50;
  public static final double WRIST_CONE_SINGLE_POSITION = 0;
  public static final double WRIST_CONE_DOUBLE_POSITION = 0;
  public static final double WRIST_CUBE_STOW_POSITION = 68.82;
  public static final double WRIST_CUBE_LOW_POSITION = -25.8;
  public static final double WRIST_CUBE_MID_POSITION = 0;
  public static final double WRIST_CUBE_HIGH_POSITION = 50;
  public static final double WRIST_CUBE_SINGLE_POSITION = 0;
  public static final double WRIST_CUBE_DOUBLE_POSITION = 0;
  public static final double WRIST_DEFAULT_STOW_POSITION = 68.82;
  public static final double WRIST_DEFAULT_LOW_POSITION = -25.8;
  public static final double WRIST_DEFAULT_MID_POSITION = 0;
  public static final double WRIST_DEFAULT_HIGH_POSITION = 50;
  public static final double WRIST_DEFAULT_SINGLE_POSITION = 0;
  public static final double WRIST_DEFAULT_DOUBLE_POSITION = 0;
    

  public static final double WRIST_TOLERANCE = 20.0;
  //Feedforward
  public static final double WRIST_G = 0.2; //Its negative because the motors move the wrong direction
  public static final double WRIST_V = 0.0;
  public static final double WRIST_A = 0.0;
  public static final double WRIST_S = 0.0;
  //PID
  public static final double WRIST_P = 0.5;
  public static final double WRIST_I = 0.0;
  public static final double WRIST_D = 0.0;

  public static final double WRIST_ENCODER_OFFSET = 3.14;
  public static final double TARGET_WRIST_ANGLE = 0;
  public static final double WRIST_ENCODER_RATIO = 1.4545;


  public RelativeEncoder wristRelativeEncoder;
  public ProfiledPIDController controller;

  private CANSparkMax wristMotor;
  private CANCoder wristCANEncoder;
  private ArmFeedforward ff;
  private double netPosition;
  private double targetWristAngle = WRIST_DEFAULT_STOW_POSITION; //TODO: make not have initialization issue
  private double voltage;

  // Logging objects
  private DataLog logger;

  /* wrist Motor Logging */
  private DoubleLogEntry wristMotorTemperature;
  private DoubleLogEntry wristMotorAppliedOutput;
  private DoubleLogEntry wristRelativeEncoderVelocity;
  //private DoubleLogEntry wristMotorBusVoltage;
  //private DoubleLogEntry wristMotorOutputCurrent;
 // private DoubleLogEntry wristMotorClosedLoopRampRate;
 // private DoubleLogEntry wristMotorOpenLoopRampRate;
  private IntegerLogEntry wristMotorFaults;
  private IntegerLogEntry wristMotorStickyFaults;
 // private StringLogEntry wristMotorIdleMode;
 // private BooleanLogEntry wristMotorInverted;


  /** Creates a new wrist. */
  public Wrist() {

    wristMotor = new CANSparkMax(WRIST_MOTOR_ID, MotorType.kBrushless);
    wristCANEncoder = new CANCoder(WRIST_ENCODER_ID);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setInverted(true);
    this.ff = new ArmFeedforward(WRIST_S, WRIST_G, WRIST_V, WRIST_A);

    wristMotor.setSmartCurrentLimit(40);

  // wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  //wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
 // wristMotor.setSoftLimit(SoftLimitDirection.kForward, wrist_FORWARD_LIMIT);
//wristMotor.setSoftLimit(SoftLimitDirection.kReverse, wrist_REVERSE_LIMIT);
  
    this.controller = new ProfiledPIDController(WRIST_P, WRIST_I, WRIST_D, new Constraints(800, 1000));
    this.controller.setTolerance(1, 1);
    wristCANEncoder.configFactoryDefault();
    wristCANEncoder.configMagnetOffset(WRIST_ENCODER_OFFSET);
    wristCANEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    wristCANEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    wristCANEncoder.configSensorDirection(true);
    
    wristRelativeEncoder = wristMotor.getEncoder();
    //wristRelativeEncoder.setPositionConversionFactor(wrist_GEAR_RATIO);
    wristRelativeEncoder.setPositionConversionFactor(WRIST_MOTOR_ROT_TO_DEG);
    wristRelativeEncoder.setVelocityConversionFactor(WRIST_MOTOR_ROT_TO_DEG);

    wristRelativeEncoder.setPosition(getPositionInDegreesCanCoder() / WRIST_MOTOR_ROT_TO_DEG);
    wristRelativeEncoder.setPositionConversionFactor(1);
    wristRelativeEncoder.setVelocityConversionFactor(WRIST_GEAR_RATIO);

    // Create logger object 
    logger = DataLogManager.getLog();
    wristMotorTemperature = new DoubleLogEntry(logger, "wristMotor/temperature");
    wristMotorAppliedOutput = new DoubleLogEntry(logger, "wristMotor/appliedOutput");
    //wristMotorBusVoltage = new DoubleLogEntry(logger, "wristMotor/busVoltage");
    //wristMotorOutputCurrent = new DoubleLogEntry(logger, "wristMotor/outputCurrent");
    //wristMotorClosedLoopRampRate = new DoubleLogEntry(logger, "wristMotor/closedLoopRampRate");
    //wristMotorOpenLoopRampRate = new DoubleLogEntry(logger, "wristMotor/openLoopRampRate");
    wristMotorFaults = new IntegerLogEntry(logger, "wristMotor/faults");
    wristMotorStickyFaults = new IntegerLogEntry (logger, "wristMotor/stickyfaults");
    wristRelativeEncoderVelocity = new DoubleLogEntry (logger, "wristMotor/velocity");
   // wristMotorIdleMode = new StringLogEntry(logger, "wristMotor/idleMode");
  //  wristMotorInverted = new BooleanLogEntry(logger, "wristMotor/inverted");
  }

  public void resetRelative() {

    wristRelativeEncoder.setPosition(getPositionInDegreesCanCoder());
  }

  public void setSpeed(double speed){
    wristMotor.set(speed);
  }

  /* Always use this method when you want the position of the wrist */
  public double getPositionInDegreesCanCoder() {
    return wristCANEncoder.getAbsolutePosition() / WRIST_ENCODER_RATIO;
  }

  public double getPositionInDegreesIntegrated() {
    return wristRelativeEncoder.getPosition();
  }

  public double getVelocityInDegreesIntegrated() {
    return wristCANEncoder.getVelocity();
  }

  /* Always use this method when you want the velocity of the wrist */
  public double getVelocityInDegreesCanCoder() {
    return wristCANEncoder.getVelocity() / WRIST_ENCODER_RATIO;
  }

  //Sets the targetWristAngle in degrees */
  public void setTargetWristAngle() {

    ModeOptions mode = RobotMode.mode;
    StateOptions state = RobotMode.state;

    if (mode == RobotMode.ModeOptions.CUBE) {

      if (state == RobotMode.StateOptions.LOW) {
        targetWristAngle = WRIST_CUBE_LOW_POSITION;
      } else if (state == RobotMode.StateOptions.MID) {
          targetWristAngle = WRIST_CUBE_MID_POSITION;
      } else if (state == RobotMode.StateOptions.HIGH) {
          targetWristAngle = WRIST_CUBE_HIGH_POSITION;
      } else if (state == RobotMode.StateOptions.SINGLE) {
          targetWristAngle = WRIST_CUBE_SINGLE_POSITION;
      } else if (state == RobotMode.StateOptions.DOUBLE) {
          targetWristAngle = WRIST_CUBE_DOUBLE_POSITION;
      } else {
          targetWristAngle = WRIST_CUBE_STOW_POSITION;
      }
  } else {
      if (state == RobotMode.StateOptions.LOW) {
          targetWristAngle = WRIST_CONE_LOW_POSITION;
      } else if (state == RobotMode.StateOptions.MID) {
          targetWristAngle = WRIST_CONE_MID_POSITION;
      } else if (state == RobotMode.StateOptions.HIGH) {
          targetWristAngle = WRIST_CONE_HIGH_POSITION;
      } else if (state == RobotMode.StateOptions.SINGLE) {
          targetWristAngle = WRIST_CONE_SINGLE_POSITION;
      } else if (state == RobotMode.StateOptions.DOUBLE) {
          targetWristAngle = WRIST_CONE_DOUBLE_POSITION;
      } else {
          targetWristAngle = WRIST_CONE_STOW_POSITION;
      }
        }


    
  }

  public void moveWristUp() {
    
    targetWristAngle = targetWristAngle + MANUAL_WRIST_SPEED;

    if (targetWristAngle >= WRIST_FORWARD_LIMIT) {
        targetWristAngle = WRIST_FORWARD_LIMIT;
    }

  }

  public void moveWristDown() {
    targetWristAngle = targetWristAngle - MANUAL_WRIST_SPEED;

    if (targetWristAngle <= WRIST_REVERSE_LIMIT) {
        targetWristAngle = WRIST_REVERSE_LIMIT;
    }
  }

  public boolean atPosition() {

    double error = Math.abs(Math.abs(getPositionInDegreesCanCoder()) - Math.abs(targetWristAngle));

    if (WRIST_TOLERANCE >= error) {

        return true;

    } else {
        return false;
    }
    
  }

  public boolean isSafeToGround() {
    return getPositionInDegreesCanCoder() < WRIST_REVERSE_LIMIT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pid = controller.calculate(getPositionInDegreesCanCoder(), targetWristAngle);
    if (DriverStation.isEnabled()){      
      setTargetWristAngle();
      double feedForward = ff.calculate(Units.degreesToRadians(getPositionInDegreesCanCoder()), Units.degreesToRadians(getVelocityInDegreesCanCoder()));
      voltage =  pid + feedForward;

      MathUtil.clamp(voltage, -12, 12);
      wristMotor.setVoltage(voltage);
      // SmartDashboard.putNumber("PID VALUE", pid);

      
    }

    /* Smart Dashboard printing */
    SmartDashboard.putNumber("CANCoder", wristCANEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("debug/wrist Position Integrated", getPositionInDegreesIntegrated());
    SmartDashboard.putNumber("debug/wrist Position Absolute", getPositionInDegreesCanCoder());
    SmartDashboard.putNumber("wrist PID Output", pid);
    // SmartDashboard.putNumber("wrist Voltage", voltage);
    SmartDashboard.putNumber("Target wrist Angle", targetWristAngle);
    SmartDashboard.putNumber("Error", Math.abs(Math.abs(getPositionInDegreesCanCoder()) - Math.abs(targetWristAngle)));
    SmartDashboard.putString("CANCoder Initialization Strategy", wristCANEncoder.configGetSensorInitializationStrategy().toString());
    
    //SmartDashboard.putNumber("NEO (Relative) Encoder", wristRelativeEncoder.getPosition());

    SmartDashboard.putNumber("Integrated Encoder", getPositionInDegreesIntegrated());

    // Logs all relevant data
    // Any additional data logging should be done in this method
    logData();
  }

  
  public Command SetWristPosition (double degrees){
    return new InstantCommand(() -> RobotMode.state = StateOptions.STOW);
  }
  

  public Command WristAtPosition(){
    return Commands.waitUntil(() -> atPosition());
  }

  private State getEncoderAngle() {
    return null;
  }

  private void logData() {
    /* wrist Motor */
    wristMotorTemperature.append(wristMotor.getMotorTemperature());
    wristMotorAppliedOutput.append(wristMotor.getAppliedOutput());
    //wristMotorBusVoltage.append(wristMotor.getBusVoltage());
    //wristMotorOutputCurrent.append(wristMotor.getOutputCurrent());
    //wristMotorClosedLoopRampRate.append(wristMotor.getClosedLoopRampRate());
    //wristMotorOpenLoopRampRate.append(wristMotor.getOpenLoopRampRate());
    wristMotorFaults.append(wristMotor.getFaults());
    wristMotorStickyFaults.append(wristMotor.getStickyFaults());
    wristRelativeEncoderVelocity.append(wristRelativeEncoder.getVelocity());

   // wristMotorIdleMode.append(wristMotor.getIdleMode().toString());
    //wristMotorInverted.append(wristMotor.getInverted());
    //wristMotorLastError.append(wristMotor.getLastError().toString());

    /* wrist CANCoder */
    // wristCANCoderAbsolutePosition.append(getPositionInDegreesCanCoder());
    // wristCANCoderAbsoluteVelocity.append(getVelocityInDegreesCanCoder());
  }
}
