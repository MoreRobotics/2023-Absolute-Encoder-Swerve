package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import frc.robot.FieldCentricOffset;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private SupplyCurrentLimitConfiguration current;
    private final Field2d m_field = new Field2d();


    // Logging objects
    private DataLog logger;
    private DoubleLogEntry robotPose2D;    

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        logger = DataLogManager.getLog();
        //Log Pose
        robotPose2D = new DoubleLogEntry(logger, "Swerve/getPose");

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        current = new SupplyCurrentLimitConfiguration(true, 0, 0, 0);

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation,
                                    getYaw().minus(Rotation2d.fromDegrees(FieldCentricOffset.getInstance().getOffset())))
                                
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(FieldCentricOffset.getInstance().getOffset());
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Rotation2d getRoll() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getRoll());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        logData();
        SmartDashboard.putNumber("debug/RobotCoordinatesX",swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("debug/RobotCoordinatesY",swerveOdometry.getPoseMeters().getY());
        //SmartDashboard.putNumber("debug/modulus yaw", Math.abs(getYaw().getDegrees() % 360));
        SmartDashboard.putNumber("debug/pitch", getRoll().getDegrees());
        SmartDashboard.putNumber("debug/yaw", getYaw().getDegrees());
        //System.out.println(getYaw().getDegrees());
        m_field.setRobotPose(swerveOdometry.getPoseMeters());
        
        Pose2d odometry = swerveOdometry.getPoseMeters();
        robotPose2D.append(odometry.getX());
        robotPose2D.append(odometry.getY());
        robotPose2D.append(odometry.getRotation().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Current Draw", mod.mDriveMotor.getSupplyCurrent());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current Draw", mod.mAngleMotor.getSupplyCurrent());
            // mod.mDriveMotor.configGetSupplyCurrentLimit(current);
            // SmartDashboard.putString("Mod " + mod.moduleNumber + " Current Limit", current.toString());
            SmartDashboard.putNumber("debug/Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("debug/Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("debug/Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 
        }
    }


    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerPath traj, boolean isFirstPath) {
        /*
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometry(traj.getPreviewStartingHolonomicPose());
                SmartDashboard.putNumber ("InitialXPos", traj.getPreviewStartingHolonomicPose().getX());
                SmartDashboard.putNumber ("InitialYPos", traj.getPreviewStartingHolonomicPose().getY());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this::getPose, // Pose supplier
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
                new PIDController(Constants.AUTO_X_P, Constants.AUTO_X_I, Constants.AUTO_X_D), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(Constants.AUTO_Y_P, Constants.AUTO_Y_I, Constants.AUTO_Y_D), // Y controller (usually the same values as X controller)
                new PIDController(Constants.AUTO_R_P, Constants.AUTO_R_I, Constants.AUTO_R_D), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this::setModuleStates, // Module states consumer
                this // Requires this drive subsystem
            )
        );
        */
        return null;
    
    }

    public Command LockWheels() {
        Translation2d stop = new Translation2d(0.0, 0.0);
        return new InstantCommand(() -> drive(stop, 90.0, true, false));
    }

    public void lockWheels() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(90)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
            new SwerveModuleState(0.0, Rotation2d.fromDegrees(90))
        });
    }
    
    private void logData() {
        for(SwerveModule mod : mSwerveMods) {
            // mod.logData();
        }
    }
}