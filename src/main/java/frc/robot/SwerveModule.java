package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.AbsoluteEncoder;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;

import javax.sound.sampled.Port;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    public CANSparkMax mAngleMotor;
    public TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder integratedAngleEncoder;

    private SparkMaxPIDController angleController;

    // private DataLog logger;
    // private DoubleLogEntry driveMotorSpeed;
    // private DoubleLogEntry driveMotorOutputCurrent;

    //private SupplyCurrentLimitConfiguration driveCurrentLimit = new SupplyCurrentLimitConfiguration(true, Constants.Swerve.drivePeakCurrentLimit,0,0);;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        /*
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();
        */

        absoluteEncoder = new AbsoluteEncoder(moduleConstants.encoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();
        
        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;

        // logger = DataLogManager.getLog();
        // driveMotorSpeed = new DoubleLogEntry(logger, "mod" + moduleNumber + "/driveMotorSpeed");
        // driveMotorOutputCurrent = new DoubleLogEntry(logger, "mod" + moduleNumber + "/driveMotorOutputCurrent");
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // SmartDashboard.putNumber("Angle Motor (pre) " + moduleNumber + ":", desiredState.angle.getDegrees());
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        // SmartDashboard.putNumber("Angle Motor (post) " + moduleNumber + ":", desiredState.angle.getDegrees());
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // Rotation2d oldAngle = getAngle();
        // angle = optimizeTurn(oldAngle, angle);  
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

        // https://www.chiefdelphi.com/t/5013-the-trobots-2023-charged-up-open-alliance-build-thread/419112/37
        public double makePositiveDegrees(double anAngle) {
            double degrees = anAngle;
            degrees = degrees % 360;
            if (degrees < 0.0){
                degrees = degrees + 360;
            }
            return degrees;
    
        }
    
        public double makePositiveDegrees(Rotation2d anAngle){
            return makePositiveDegrees(anAngle.getDegrees());
        }
    
        public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle){
            double steerAngle = makePositiveDegrees(newAngle);
            steerAngle %= (360);
            if (steerAngle < 0.0) {
                steerAngle += 360;
            }
    
            double difference = steerAngle - oldAngle.getDegrees();
            // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
            if (difference >= 360) {
                steerAngle -= 360;
            } else if (difference < -360) {
                steerAngle += 360;
            }
            difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference
    
            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference >90 || difference < -90) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += 180;
            }
    
            return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
        }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    
    public Rotation2d getCanCoder(){
        
        //return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        
        return absoluteEncoder.getRotation();
    }


    public void resetToAbsolute(){
        double absolutePosition = absoluteEncoder.getRotation().getDegrees() - angleOffset.getDegrees(); 
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        // angleEncoder.configFactoryDefault();
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKF);
        mAngleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
        //mDriveMotor.configSupplyCurrentLimit(driveCurrentLimit);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public void logData() {
        //driveMotorSpeed.append(mDriveMotor.getSelectedSensorVelocity());
        //driveMotorOutputCurrent.append((mDriveMotor.getStatorCurrent()));
    }
}