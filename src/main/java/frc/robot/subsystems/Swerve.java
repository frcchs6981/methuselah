package frc.robot.subsystems;

import frc.lib.util.SwerveModule;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.concurrent.locks.ReentrantLock;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

//import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public ADIS16470_IMU gyro;

    private final ReentrantLock swerveModLock = new ReentrantLock();

    public RobotConfig config;
    

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
        swerveModLock.unlock();
    }

    public double getYaw() {
        return gyro.getAngle(); 
    }

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                getChassisSpeeds()
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

public void runSetupPhase()
{
    try{ config = RobotConfig.fromGUISettings(); } 
    catch (Exception e) { e.printStackTrace(); }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            (pose) -> setPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive(speeds, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    
        );
}


public ChassisSpeeds getChassisSpeeds() 
{
    // ChassisSpeeds speeds = new ChassisSpeeds(
    //   translation.getX(), 
    //   translation.getY(), 
    //   rotation
    //   );
    // ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds( 0, gyro.getRate(), getGyroYaw())
       /* // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        // Front left module state
        SwerveModuleState frontLeft = moduleStates[0];
        // Front right module state
        SwerveModuleState frontRight = moduleStates[1];
        // Back left module state
        SwerveModuleState backLeft = moduleStates[2];
        // Back right module state
        SwerveModuleState backRight = moduleStates[3];*/

        double speedHypot = getModuleStates()[0].speedMetersPerSecond 
        + getModuleStates()[1].speedMetersPerSecond 
        + getModuleStates()[2].speedMetersPerSecond 
        + getModuleStates()[3].speedMetersPerSecond / 4.0;

        double vx = speedHypot * Math.cos(Units.degreesToRadians(gyro.getAngle())); 
        double vy = speedHypot* Math.sin(Units.degreesToRadians(gyro.getAngle())); 

        return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(
            vx, 
            vy, 
            Units.degreesToRadians(gyro.getRate())), 
            Rotation2d.fromDegrees(gyro.getAngle())
        ); 
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
    public void periodic()
    {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        swerveModLock.lock();
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}