package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter ;
 
public class ShooterRollers extends Command {
 
  public double speed;
  public double pos;
 
 
 
 
 
  public ShooterRollers(double speed) {
    this.speed = speed;
 
  }
 
  @Override
  public void initialize() {
    // lowerStopped = false;
 
  }
 
  @Override
  public void execute()
  {
   
    Shooter.setRollers(speed);
 
    // if(Intake.getUpperBeamBreak())
    // {
    //   upperStopped = true;
    // }
    // if(!upperStopped)
    // {
    //   Intake.setRollers(speed, 3);
    // }
    // else{
    //   Intake.setRollers(speed, 4);
    // }
  }
    // else{
    //   System.out.println(Intake.centerIntakeRoller.getPosition().refresh().getValueAsDouble());
    //   if (isHomeSet == false){
    //     Intake.centerIntakeRoller.setPosition(0);
    //     isHomeSet = true;
    //   }
     
    //       if (Intake.centerIntakeRoller.getPosition().refresh().getValueAsDouble() < 50.0){
    //         Intake.setRollers(speed);
    //       }
    //       else{
 
    //         Intake.setRollers(0);
    //       }
    //       }
    //     }
 
 
 
 
  @Override
  public void end(boolean interrupted) {
  // if(Intake.getLowerBeamBreak()==false){
  //   Intake.setRollers(0,2);
  // }
  Shooter.setRollers(0);
  }
 
  @Override
  public boolean isFinished() {
   
    return false;
   
  }
}