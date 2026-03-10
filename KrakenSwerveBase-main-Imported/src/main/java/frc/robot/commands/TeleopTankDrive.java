// package frc.robot.commands;

// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;


// public class TeleopTankDrive extends Command {    
//     private Swerve s_Swerve;    
//     private DoubleSupplier LeftSup;
//     private DoubleSupplier RightSup;

//     public TeleopTankDrive(Swerve s_Swerve, DoubleSupplier LeftSup, DoubleSupplier RightSup) {
//         this.s_Swerve = s_Swerve;
//         addRequirements(s_Swerve);

//         this.LeftSup = LeftSup;
//         this.RightSup = RightSup;
//     }

//     @Override
//     public void execute() {
//         /* Get Values, Deadband*/
//         double LeftVal = MathUtil.applyDeadband(LeftSup.getAsDouble(), Constants.stickDeadband);
//         double RightVal = MathUtil.applyDeadband(RightSup.getAsDouble(), Constants.stickDeadband);

//         /* Drive */
//         s_Swerve.tankdrive(
//  LeftVal, RightVal
//         );
//     }
// }