// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Timer;
/*travel certain distances to pick balls  up and also intake
and turn degrees

network tables closest ball is on right or left
if right:
  Path A
if left:
  Path B

motor encoder returns distance traveled
motor.set(0.5,0.5)
if(distance>target){
  motor.set(0.0,0.0)
  encoder.set
}

navx returns current angle

if(currentangle > angle){
  motor.set(0.0, 0.0)
}else{
  motor.set(-0.5,0.5)
}
INTAKE*/
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private Joystick stick;
  private final Timer m_timer = new Timer();
  private VictorSPX motorFrontL, motorFrontR, motorBackL, motorBackR; 
  private VictorSP shooter;
  private VictorSPX elevator, intake;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //JOYSTICK in PORT 1, check FRC driver station
    stick = new Joystick(1);

    //DRIVING MOTORS
    motorFrontR = new VictorSPX(0);
    motorFrontL = new VictorSPX(1);
    motorBackL = new VictorSPX(2);
    motorBackR = new VictorSPX(3);

    //SHOOTER MOTOR
    shooter = new VictorSP(4);

    //ELEVATOR MOTOR
    elevator = new VictorSPX(5);

    //INTAKE MOTOR
    intake = new VictorSPX(6);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    System.out.println(m_timer.get());
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

    //SHOOTER
    //R1 to shoot
    //Y to stop
    if (stick.getRawButton(6)){
      shooter.set(-0.80);
    }
    if(stick.getRawButton(4)){
      shooter.set(0);
    }

    //ELEVATOR
    //L1 to Elevator
    if (stick.getRawButton(5)) {
      elevator.set(ControlMode.PercentOutput, -0.55);
    } else {
      elevator.set(ControlMode.PercentOutput, 0);
    }

    //INTAKE
    //A to intake
    //B to reverse intake if Ball is stuck
    if (stick.getRawButton(1)) {
      intake.set(ControlMode.PercentOutput, 0.40);
    } else if (stick.getRawButton(2)) {
      intake.set(ControlMode.PercentOutput, -0.40);
    } else {
      intake.set(ControlMode.PercentOutput, 0);
    }

    //DRIVING CODE
    double x = 0, y = 0, z = 0;

    if (!(stick.getX() < 0.2 && stick.getX() > -0.2)) {
      z = stick.getX();
    }
    if (!(stick.getY() < 0.2 && stick.getY() > -0.2)) {
      x = -stick.getY();
    }
    if (!(stick.getRawAxis(4) < 0.2 && stick.getRawAxis(4) > -0.2)) {
      y = stick.getRawAxis(4);
    }

    double[] w = new double[4];
    w[0] = x + y + z;
    w[1] = x - y - z;
    w[2] = x - y + z;
    w[3] = x + y - z;

    double wMax = 1;

    for (int i = 0; i < 4; i++) {
      if (w[i] > wMax) {
        wMax = w[i];
      }
    }
    
    motorFrontL.set(ControlMode.PercentOutput, maxSpeedCheck(w[0] / (wMax*1.7 )));
    motorFrontR.set(ControlMode.PercentOutput, -maxSpeedCheck(w[1] / (wMax*1.7 )));
    motorBackL.set(ControlMode.PercentOutput, maxSpeedCheck(w[2] / (wMax*1.7 )));
    motorBackR.set(ControlMode.PercentOutput, -maxSpeedCheck(w[3] / (wMax*1.7 )));

    //X to move shooter for 3 seconds
    if(stick.getRawButton(3)){
      m_timer.reset();
      m_timer.start();
      shooter.set(-0.5);
    }

    if(m_timer.get() >= 3.0){
      m_timer.stop();
      m_timer.reset();
      shooter.set(0);
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public double maxSpeedCheck(double inputSpeed) {
    if (inputSpeed > 1) {
      return 1.0;
    } else {
      return inputSpeed;
    }
  }
}
