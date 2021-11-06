// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//importations
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.kauailabs.navx.frc.AHRS;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  // Declaring Motors
  private CANSparkMax frontRightMotor;
  private CANSparkMax backRightMotor;
  private CANSparkMax frontLeftMotor;
  private CANSparkMax backLeftMotor;

  // Declaring encoders
  private CANEncoder frontRightEncoder;
  private CANEncoder backRightEncoder;
  private CANEncoder frontLeftEncoder;
  private CANEncoder backLeftEncoder;

  // Declaring Mecanum Drive
  private MecanumDrive mecanumDrive;

  // Declares NavX
  private AHRS NavX;

  // declares Field Orientation Boolean
  boolean fieldOriented;

  // used for slow mode and turbo mode
  private double speedScalar;
  

  public DriveSubsystem() 
  {
    //Instantiates Drive Motors
    frontRightMotor = new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(DriveConstants.BACK__RIGHT_MOTOR, MotorType.kBrushless);
    frontLeftMotor= new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(DriveConstants.BACK_LEFT_MOTOR, MotorType.kBrushless);

    //Instantiates Mecanum Drive
    mecanumDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  

    //Sets current Limit
    frontRightMotor.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
    backRightMotor.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
    frontLeftMotor.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
    backLeftMotor.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);

    //Sets Voltage Compensation
    frontRightMotor.enableVoltageCompensation(DriveConstants.VOLTAGE_COMP);
    backRightMotor.enableVoltageCompensation(DriveConstants.VOLTAGE_COMP);
    frontLeftMotor.enableVoltageCompensation(DriveConstants.VOLTAGE_COMP);
    backLeftMotor.enableVoltageCompensation(DriveConstants.VOLTAGE_COMP);
    
    //Sets Ramp Rate
    frontRightMotor.setOpenLoopRampRate(DriveConstants.RAMP_RATE);
    backRightMotor.setOpenLoopRampRate(DriveConstants.RAMP_RATE);
    frontLeftMotor.setOpenLoopRampRate(DriveConstants.RAMP_RATE);
    backLeftMotor.setOpenLoopRampRate(DriveConstants.RAMP_RATE);

    //Sets PID Values for Drive Motors
    setPID(frontRightMotor);
    setPID(backRightMotor);
    setPID(frontLeftMotor);
    setPID(backLeftMotor);

  }
  
  //configures PID values for the motor in the constructor
  public void setPID(CANSparkMax motor) {
    CANPIDController pid = motor.getPIDController();
    pid.setP(DriveConstants.KP);
    pid.setI(DriveConstants.KI);
    pid.setD(DriveConstants.KD);
    pid.setIZone(DriveConstants.KIZ);
    pid.setFF(DriveConstants.KFF);
    pid.setOutputRange(DriveConstants.K_MIN_OUTPUT, DriveConstants.K_MAX_OUTPUT);
    motor.setSmartCurrentLimit(DriveConstants.CURRENT_LIMIT);
    motor.enableVoltageCompensation(DriveConstants.VOLTAGE_COMP);
  }

  //Method that allows the robot to drive
  public void drive(double xSpeed, double ySpeed, double zSpeed) {
   if (fieldOriented) {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zSpeed, NavX.getAngle());
   } else {
    mecanumDrive.driveCartesian(ySpeed, xSpeed, zSpeed);
   }
  }

  //Method that allows for Field Oriented driving to be toggled
  public void toggleFieldOriented(){
    fieldOriented = !fieldOriented;
   }

   //sets the speedScalar
   public void setSpeedScalar(double scalar){
    speedScalar = scalar;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is field oriented", fieldOriented);
  }
}
