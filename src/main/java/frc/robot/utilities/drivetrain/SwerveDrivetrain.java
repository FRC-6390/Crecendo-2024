// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.utilities.controlloop.PID;
import frc.robot.utilities.controlloop.PIDConfig;
import frc.robot.utilities.swerve.SwerveModule;

/** Add your docs here. */
public class SwerveDrivetrain {

    private static SwerveModule[] swerveModules;
    private static Pigeon2 gyro;
    private static SwerveDriveKinematics kinematics;
    private static PIDConfig driftCorrectionConfig;
    private ChassisSpeeds speeds;
    private ChassisSpeeds addedSpeeds;
    private static boolean shouldCorrect;
    private double desiredHeading; 
    private PID driftpid;

    public SwerveDrivetrain(SwerveModule[] modules, int gyro) 
    {
    this(modules, 1, true, new PIDConfig(0, 0, 0));
    }

    public SwerveDrivetrain(SwerveModule[] modules, int gyroPigeon2, boolean driftCorrection, PIDConfig driftCorrectionConfigPidConfig) 
    {
        for (int i = 0; i < modules.length; i++) {
            swerveModules[i] = modules[i];
        }
        gyro = new Pigeon2(gyroPigeon2);
        Translation2d[] moduleLocations = new Translation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleLocations[i] = modules[i].getModuleLocation();
        }
        kinematics = new SwerveDriveKinematics(moduleLocations);
        driftCorrectionConfig = driftCorrectionConfigPidConfig;
        shouldCorrect = driftCorrection;
        driftpid = new PID(driftCorrectionConfigPidConfig);
    }

  public double getRate(){
    return gyro.getRate();
  }

  public void resetHeading(){
    gyro.setYaw(0);
  }

  public double getRoll(){
    return Math.IEEEremainder(gyro.getRoll().refresh().getValueAsDouble(), 360);
  }

  public double getPitch(){
    return Math.IEEEremainder(gyro.getPitch().refresh().getValueAsDouble(),360);
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getYaw().refresh().getValueAsDouble(), 360);
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

    private void setModuleStates(SwerveModuleState[] states)
    {
        for (int i = 0; i < states.length; i++) {
            swerveModules[i].setDesiredState(states[i]);
        }
    }

    public void lockWheels()
    {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].lock();
        }
    }

    public void unlockWheels()
    {
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].unlock();
        }
    }

    public void drive(ChassisSpeeds driveSpeeds)
    {
        speeds = driveSpeeds;
    }

    public void addSpeed(ChassisSpeeds addedSpeeds)
    {
        this.addedSpeeds = addedSpeeds;
    }

    public void driftCorrection(ChassisSpeeds speeds)
    {
        if(Math.abs(speeds.omegaRadiansPerSecond) > 0.0) 
        {desiredHeading = getHeading();}
        else {speeds.omegaRadiansPerSecond += driftpid.calculate(desiredHeading);};
    }

    public void update()
    {
        ChassisSpeeds speed = speeds.plus(addedSpeeds);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speed);

        if(shouldCorrect)
        {
            driftCorrection(speed);
        }

        setModuleStates(states);
       
    }

    public void enableDriftCorrection()
    {
        shouldCorrect = true;
    }

    public void disableDriftCorrection()
    {
        shouldCorrect = false;
    }

}
