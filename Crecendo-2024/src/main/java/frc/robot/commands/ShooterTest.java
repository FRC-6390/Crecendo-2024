// package frc.robot.commands;

// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.motorcontrol.NeutralMode;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// //import frc.robot.subsystems.Arm;

// public class Shooter extends Command {
 
//   public double speed;
//   public boolean isDone;
//   public String gamePiece;
//   public double rotations;

//   public Shooter(double speed, String gamePiece, double rotations) {
//     this.speed = speed;
//    this.gamePiece = gamePiece;
//     this.rotations = rotations;
//   }

  
//   @Override
//   public void initialize() {
//     isDone = false;
//     Arm.outputRoller.setNeutralMode(NeutralMode.Brake);
//   }

  

   
//   @Override
  
//   public void end(boolean interrupted) {
//     //Stops the motors
//     Arm.setRoller(0);
//   }

  
//   @Override
//   public boolean isFinished() {
//     return isDone;
//   }
// }
