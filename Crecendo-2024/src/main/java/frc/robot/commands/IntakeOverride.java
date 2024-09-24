package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.controller.DebouncedJoystick;
 
public class IntakeOverride extends Command {
 
  public double speed;
  public double pos;
 
  public boolean lowerStopped = false;
  public static boolean upperStopped = false;
  public Intake intake = new Intake();
  public boolean isDone = false;
 
  public IntakeOverride(double speed, Intake intake) {
    this.speed = speed;

    this.intake = intake;
    addRequirements(intake);
  }
 
  @Override
  public void initialize() {

    lowerStopped = false;
   upperStopped = false;
   isDone = false;
 
  }
 
  @Override
  public void execute()
  {
  intake.centerIntake(speed/3);
  intake.fullWidthMove(speed);
  intake.feed(speed/3);
  // if(DriverStation.isTeleop()){

  //  upperStopped = false;
  //  if(Intake.getUpperBeamBreak())
  //   {
  //     upperStopped = true;
  //   }
  //   if(!upperStopped)
  //   {
  //     intake.setRollers(speed, 2);

  //   }
  //   else
  //   {
  //     intake.setRollers(speed, 1);
  //   }
  //   }

  //   if(DriverStation.isAutonomous()){

  //     upperStopped = false;
  //     if(Intake.getUpperBeamBreak())
  //      {
  //        upperStopped = true;
  //      }
  //      if(!upperStopped)
  //      {
  //        intake.setRollers(speed, 2);
  //      }
  //      else
  //      {
  //       isDone = true;
  //      }
  //      }  
   
  }

 
 
  @Override
  public void end(boolean interrupted) {
 
  intake.setRollers(0, 1);
  }
 
  @Override
  public boolean isFinished() {
   if(DriverStation.isTeleop())
   {
    return false;
   }
   else
   {
    return isDone;
   }
   
  }
}