// package frc.robot.commands;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.utilities.controller.DebouncedJoystick;
 
// public class Reverse extends Command {
 
//   public double speed;
//   public double pos;
 
//   public boolean lowerStopped = false;
//   public static boolean upperStopped = false;
//   public Intake intake = new Intake();


 
 

 
 
//   public Reverse(double speed, Intake intake) {
//     this.speed = speed;

//     this.intake = intake;
//     addRequirements(intake);
//   }
 
//   @Override
//   public void initialize() {

//     lowerStopped = false;
//    upperStopped = false;
 
//   }
 
//   @Override
//   public void execute()
//   {
//   intake.setRollers(speed, 2);
   
//   }

 
 
//   @Override
//   public void end(boolean interrupted) {
 
//   intake.setRollers(0, 1);
//   }
 
//   @Override
//   public boolean isFinished() {
   
//     return false;
   
//   }
// }