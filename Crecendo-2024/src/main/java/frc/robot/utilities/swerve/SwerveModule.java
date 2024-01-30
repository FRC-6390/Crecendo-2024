package frc.robot.utilities.swerve;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.SWERVEMODULE;
import frc.robot.utilities.controlloop.PID;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX rotationMotor;

    private CANcoder encoder;
    private PID pid;
    
    private GenericEntry offsetEntry;

    private double encoderOffset;
   // private REVMaglimitSwitch limitSwitch;
    private static int instances = 0;

    public SwerveModule(SwerveModuleConfig config){
        this(config, null);
    }

    private static PIDController rotationPidController = new PIDController(0.35, 0.1, 0);
    


    public SwerveModule(SwerveModuleConfig config, ShuffleboardTab tab){
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        if(config.canbus() != null){
            System.out.println(config.canbus());
            driveMotor = new TalonFX(config.driveMotor(), config.canbus());
            rotationMotor = new TalonFX(config.rotationMotor(), config.canbus());
            encoder = new CANcoder(config.encoder(), config.canbus());
        }else{
            driveMotor = new TalonFX(config.driveMotor());
            rotationMotor = new TalonFX(config.rotationMotor());
            encoder = new CANcoder(config.encoder());
        }
        encoderOffset = config.encoderOffset();
        driveMotor.setInverted(config.driveMotorReversed());
        rotationMotor.setInverted(config.rotationMotorReversed());

        CANcoderConfiguration  con = new CANcoderConfiguration();
        con.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        pid = new PID(SWERVEMODULE.ROTATION_PID).setMeasurement(() -> getRotationMotorPosition());
        if(tab != null){
            ShuffleboardLayout layout = tab.getLayout("Swerve Module "+instances, BuiltInLayouts.kList).withSize(2, 6);
            layout.add(pid);
            offsetEntry = layout.add("Offset "+ instances, getEncoderOffset()).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -Math.PI, "max", Math.PI)).getEntry();
            layout.addDouble("Angle "+instances, () -> getPostion().angle.getDegrees());
            layout.addDouble("Absolute "+instances, () -> getAbsolutePosition());
        }
        instances++;

        resetEncoders();
        lock();
    }

    public double getDriveMotorVelocity(){
        // .getSensorCollection().getIntegratedSensorVelocity() this is not good as it does not match the CAN frame aparently
        return driveMotor.getVelocity().refresh().getValueAsDouble() / 2048 /2*Math.PI * SWERVEMODULE.DRIVE_ENCODER_CONVERSION_METERS;
        
    }
    
    public double getDriveMotorPosition(){
        // .getSensorCollection().getIntegratedSensorPosition() this is not good as it does not match the CAN frame aparently
        return driveMotor.getPosition().refresh().getValueAsDouble() / 2048 /2*Math.PI * SWERVEMODULE.DRIVE_ENCODER_CONVERSION_METERS;
    }

    public double getRotationMotorPosition(){
        return getEncoderRadians();
    }

    public double getAbsolutePosition(){
        return encoder.getAbsolutePosition().refresh().getValueAsDouble() * Math.PI/180d;
    }

    public double getEncoderOffset(){
        return encoderOffset;
    }

    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    public double getEncoderRadians(){
        if(offsetEntry != null) encoderOffset = offsetEntry.getDouble(0.0);

        return (encoder.getAbsolutePosition().refresh().getValueAsDouble() * Math.PI/180d) - encoderOffset;
    }

    public void resetEncoders(){
        driveMotor.setPosition(0);
        rotationMotor.setPosition(getEncoderRadians());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getEncoderRadians()));
    }

    public SwerveModulePosition getPostion(){
        return new SwerveModulePosition(getDriveMotorPosition(), new Rotation2d(getEncoderRadians()));
    }

    public void setDriveMotor(double speed){
        driveMotor.set(speed);
    }

    public void setRotationMotor(double speed){
        rotationMotor.set(speed);
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / SWERVEMODULE.MAX_SPEED_METERS_PER_SECOND);
        // rotationMotor.set(ControlMode.PercentOutput, pid.calculate(state.angle.getRadians()));
        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void setToAngle(double angle){
        SwerveModuleState state = new SwerveModuleState(0, new Rotation2d(angle));
        //setDesiredState(state);
        rotationMotor.set(rotationPidController.calculate(-getEncoderRadians(), -state.angle.getRadians()));
    }

    public void lock(){
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void unlock(){
        driveMotor.setNeutralMode(NeutralModeValue.Coast);
        rotationMotor.setNeutralMode(NeutralModeValue.Coast);
    }

}