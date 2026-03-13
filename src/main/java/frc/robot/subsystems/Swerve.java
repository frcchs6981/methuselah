package frc.robot.subsystems;

import frc.lib.util.SwerveModule;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.concurrent.locks.ReentrantLock;

//import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public ADIS16470_IMU gyro;

    private final ReentrantLock swerveModLock = new ReentrantLock();

    public Swerve() {
        //Creates Gyro and sets used angle (Z) to 0
        gyro = new ADIS16470_IMU();
        gyro.setGyroAngleZ(0);
        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants, CTREConfigs.getFLCANCoderConf()),
            new SwerveModule(1, Constants.Swerve.Mod1.constants,CTREConfigs.getFRCANCoderConf()),
            new SwerveModule(2, Constants.Swerve.Mod2.constants,CTREConfigs.getBLCANCoderConf()),
            new SwerveModule(3, Constants.Swerve.Mod3.constants,CTREConfigs.getBRCANCoderConf())
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? 
                                ChassisSpeeds.fromFieldRelativeSpeeds( // Drives Field Relative
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                              : new ChassisSpeeds( // Drives Robot Relative
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        
        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
        swerveModLock.unlock();
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        swerveModLock.unlock();

        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        swerveModLock.unlock();

        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public Command zeroHeading(){
        return Commands.runOnce(
            () -> {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
            }, 
            this);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Command resetModulesToAbsolute(){
        return Commands.runOnce(
            () -> {
                swerveModLock.lock();
                for(SwerveModule mod : mSwerveMods){
                    mod.resetToAbsolute();
                }
                swerveModLock.unlock();
            }, 
            this);
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        swerveModLock.unlock();
    }
}