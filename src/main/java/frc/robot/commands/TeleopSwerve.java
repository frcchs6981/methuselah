package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
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

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.SpeedSup = SpeedSup;
    }

    @Override
    public void execute() {
        //Uses values from TeleopSwerve() to drive
        
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(-translationSup.getAsDouble(), Constants.stickDeadband ) * (1 - SpeedSup.getAsDouble());
        double strafeVal = MathUtil.applyDeadband(-strafeSup.getAsDouble(), Constants.stickDeadband) * (1 - SpeedSup.getAsDouble());
        double rotationVal = MathUtil.applyDeadband(-rotationSup.getAsDouble(), Constants.stickDeadband) * (1 - SpeedSup.getAsDouble());

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}