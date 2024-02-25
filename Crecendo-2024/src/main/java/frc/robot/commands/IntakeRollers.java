package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
 
public class IntakeRollers extends Command {
 
  public double speed;
  public double pos;
 
  public boolean lowerStopped = false;
  public boolean upperStopped = false;
  public Intake intake = new Intake();
 
 

 
 
  public IntakeRollers(double speed, Intake intake) {
    this.speed = speed;
    this.intake = intake;
    addRequirements(intake);
  }
 
  @Override
  public void initialize() {

    lowerStopped = false;
   upperStopped = false;
 
  }
 
  @Override
  public void execute()
  {
   //System.out.print("_____________________"+Intake.getUpperBeamBreak()+"---------------------------------");
   upperStopped = false;
   if(Intake.getUpperBeamBreak())
    {
      upperStopped = true;
    }
    if(!upperStopped)
    {
      intake.setRollers(speed, 2);
     // Intake.setRollers(speed, 4);
    }
    else
    {
      intake.setRollers(speed, 1);
     // Intake.setRollers(speed, 3);
    }
 
   
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
 
 
 
 
  @Override
  public void end(boolean interrupted) {
  // if(Intake.getLowerBeamBreak()==false){
  //   Intake.setRollers(0,2);
  // }
  intake.setRollers(0, 2);
  }
 
  @Override
  public boolean isFinished() {
   
    return false;
   
  }
}