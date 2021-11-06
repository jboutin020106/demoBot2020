// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeAndOuttakeConstants;;

public class IntakeAndOuttake extends SubsystemBase {
  
  //declares motors and PID Controllers
  private CANSparkMax shooterLeader;
  private CANSparkMax shooterFollower;
  private WPI_VictorSPX conveyorMotor1;
  private WPI_VictorSPX conveyorMotor2;
  private CANPIDController leaderController;
  private WPI_TalonSRX intakeMotor;
  private DigitalInput sensor1;
  private DigitalInput sensor2;
  private DigitalInput sensor3;

  private double targetVelocity;


  /** Creates a new IntakeAndOuttake. */
  public IntakeAndOuttake() {
    //Instantiates motors and controllers
    shooterLeader = new CANSparkMax(IntakeAndOuttakeConstants.SHOOTER_LEADER, MotorType.kBrushless);
    shooterFollower = new CANSparkMax(IntakeAndOuttakeConstants.SHOOTER_FOLLOWER, MotorType.kBrushless);
    conveyorMotor1 = new WPI_VictorSPX(IntakeAndOuttakeConstants.CONVEYOR_MOTOR_1);
    conveyorMotor2 = new WPI_VictorSPX(IntakeAndOuttakeConstants.CONVEYOR_MOTOR_2);
    intakeMotor = new WPI_TalonSRX(IntakeAndOuttakeConstants.INTAKE_PRIMARY);
    leaderController = shooterLeader.getPIDController();
    sensor1 = new DigitalInput(IntakeAndOuttakeConstants.INTAKE_SENSOR_1);
    sensor2 = new DigitalInput(IntakeAndOuttakeConstants.INTAKE_SENSOR_2);
    sensor3 = new DigitalInput(IntakeAndOuttakeConstants.INTAKE_SENSOR_3);
    

    // sets the motor to always run at the same speed as shooterLeader
    shooterFollower.follow(shooterLeader);

    //sets the PID values for the leaderController
    leaderController.setP(IntakeAndOuttakeConstants.KP);
    leaderController.setI(0);
    leaderController.setD(IntakeAndOuttakeConstants.KD);
    leaderController.setIZone(IntakeAndOuttakeConstants.KIZ);
    leaderController.setFF(IntakeAndOuttakeConstants.KFF);
    leaderController.setOutputRange(IntakeAndOuttakeConstants.K_MIN_OUTPUT, IntakeAndOuttakeConstants.K_MAX_OUTPUT);

    //sets up the shooterLeader and shooterFollower
    shooterLeader.setInverted(true);
    shooterLeader.setSmartCurrentLimit(IntakeAndOuttakeConstants.CURRENT_LIMIT);
    shooterLeader.enableVoltageCompensation(IntakeAndOuttakeConstants.VOLTAGE_COMP);
    shooterLeader.setOpenLoopRampRate(IntakeAndOuttakeConstants.RAMP_RATE);
    shooterFollower.setSmartCurrentLimit(IntakeAndOuttakeConstants.CURRENT_LIMIT);
    shooterFollower.enableVoltageCompensation(IntakeAndOuttakeConstants.VOLTAGE_COMP);
    shooterFollower.setOpenLoopRampRate(IntakeAndOuttakeConstants.RAMP_RATE);

  }
  
  public void shoot (double rpm){
    leaderController.setReference(rpm, ControlType.kVelocity);
    load();
  }

  //allows the robot to load power cells
  public void load(){
    boolean upToSpeed = (getShooterVelocity() + 45 > targetVelocity) && (getShooterVelocity() - 45 < targetVelocity);
    if(!upToSpeed && sensor3.get()){
      conveyorMotor2.set(0.0);
      conveyorMotor1.set(0.0);
    }
    else{
      conveyorMotor2.set(0.6);
      conveyorMotor1.set(0.6);
    }
  
    
    
  }

  // allows the robot to intake power cells
  public void intake(){
    intakeMotor.set(0.6);
    conveyorMotor1.set(0.6);
  }

  // allows the robot to reverse the Intake system
  public void reverseIntake(){
    intakeMotor.set(-0.6);
    conveyorMotor1.set(-0.6);
  }

  private double getShooterVelocity(){
    return shooterLeader.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
