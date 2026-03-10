package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public static CANcoderConfiguration getFLCANCoderConf() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        config.MagnetSensor.MagnetOffset = -0.306885; 
        return config; 
    }
    public static CANcoderConfiguration getFRCANCoderConf() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        config.MagnetSensor.MagnetOffset = 0.435547; 
        return config; 
    }
    public static CANcoderConfiguration getBLCANCoderConf() {
       CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        config.MagnetSensor.MagnetOffset = 0.083984 ;
        return config; 
    }
    public static CANcoderConfiguration getBRCANCoderConf() {
                CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        config.MagnetSensor.MagnetOffset = -0.250000; 
        return config; 
    }

    public static TalonFXConfiguration getDriveConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        config.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        config.MotorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;

        /* Gear Ratio Config */
        config.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentThreshold;
        config.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;

        /* PID Config */
        config.Slot0.kP = Constants.Swerve.driveKP;
        config.Slot0.kI = Constants.Swerve.driveKI;
        config.Slot0.kD = Constants.Swerve.driveKD;

        /* Open and Closed Loop Ramping */
        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;

        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        return config; 
    }
    public static TalonFXConfiguration getSteerConfig()
    {
        TalonFXConfiguration config = new TalonFXConfiguration(); 
        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        config.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        config.MotorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        config.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.angleCurrentThreshold;
        config.CurrentLimits.SupplyCurrentLowerTime  = Constants.Swerve.angleCurrentThresholdTime;

        /* PID Config */
        config.Slot0.kP = Constants.Swerve.angleKP;
        config.Slot0.kI = Constants.Swerve.angleKI;
        config.Slot0.kD = Constants.Swerve.angleKD;
        return config; 
    }
}