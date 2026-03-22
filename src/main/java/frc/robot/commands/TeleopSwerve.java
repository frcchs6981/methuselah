package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private DoubleSupplier SpeedSup;

    //Inputs values from RobotContainer.java regarding movement inputs and Field Relative
    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, DoubleSupplier SpeedSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup; //gp 1
        this.strafeSup = strafeSup; //gp 0
        this.rotationSup = rotationSup; //gp 4
        this.robotCentricSup = robotCentricSup; //false
        this.SpeedSup = SpeedSup; //gp 3
    }

    @Override
    public void execute() {
        //Uses values from TeleopSwerve() to drive
        
        /* Get Values, Deadband*/
        double CoolHeckinMult = 1 - (0.721738 * Math.log10(16.66 * SpeedSup.getAsDouble()));
        double translationVal = MathUtil.applyDeadband(-translationSup.getAsDouble(), Constants.stickDeadband ) * CoolHeckinMult;
        double strafeVal = MathUtil.applyDeadband(-strafeSup.getAsDouble(), Constants.stickDeadband) * CoolHeckinMult;
        double rotationVal = MathUtil.applyDeadband(-rotationSup.getAsDouble(), Constants.stickDeadband) * CoolHeckinMult;

        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(translationVal, strafeVal, rotationVal), Rotation2d.fromDegrees(RobotContainer.getSwerve().getYaw())); 
        /* Drive */
        s_Swerve.drive(
            speeds, 
            true
        );
    }
}