package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

  private Joystick stick;
  private final Timer mytimer = new Timer();
  private TalonSRX elevator, intake; 
  private TalonSRX FrontL, FrontR, BackL, BackR;
  private VictorSP shooter;
  private double shooterSpeed = -0.65;
  private AHRS ahrs;

  private double distance;
  private int orientation;
  //if turn:
  // + deg is right
  // - deg is left
  private AutonomousCommands[] autonomousIntakeChallenge = new AutonomousCommands[]{
    new AutonomousCommands("move", 150.0 + 21.0),
    new AutonomousCommands("turn", -71),
    new AutonomousCommands("move", 94.9 + 18.0),
    new AutonomousCommands("turn", 98.2),
    new AutonomousCommands("move", 134.2 + 21.0),
  };
  private int currentCommand;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //JOYSTICK in PORT 1, check FRC driver station
    stick = new Joystick(1);

    //DRIVING MOTORS
    FrontR = new TalonSRX(3);
    FrontL = new TalonSRX(1);
    BackL = new TalonSRX(0);
    BackR = new TalonSRX(2);

    //SHOOTER MOTOR
    shooter = new VictorSP(4);

    //ELEVATOR MOTOR
    elevator = new TalonSRX(4);

    //INTAKE MOTOR
    intake = new TalonSRX(5);

    //AHRS
    ahrs = new AHRS(SPI.Port.kMXP); 

    //NETWORK TABLES

    //get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    //get a reference to the subtable called "datatable"
    NetworkTable table = inst.getTable("Vision");

    //get a reference to key in "datatable" called "Y"
    NetworkTableEntry orientationEntry = table.getEntry("orientation");

    //add an entry listener for changed values of "X", the lambda ("->" operator)
    //defines the code that should run when "X" changes
    table.addEntryListener("distance", (tab, key, entry, value, flags) -> {
        distance = Double.parseDouble(value.getValue().toString());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    //add an entry listener for changed values of "Y", the lambda ("->" operator)
    //defines the code that should run when "Y" changes
    orientationEntry.addListener(event -> {
        orientation=Integer.parseInt(event.value.getValue().toString());
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    ahrs.zeroYaw();
    FrontR.setSelectedSensorPosition(0.0);
    currentCommand = 0;
    mytimer.reset();
    mytimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    intake.set(ControlMode.PercentOutput, 0.40);
    if(currentCommand < autonomousIntakeChallenge.length){
      if(autonomousIntakeChallenge[currentCommand].getCommandType() == "move"){
        if(unitToDistance(FrontR.getSelectedSensorPosition()) < autonomousIntakeChallenge[currentCommand].getValue()){
          moveMotors(0.3, 0.3);
        } else {
          currentCommand++;
          FrontR.setSelectedSensorPosition(0.0);
        }
      } else {
        if(autonomousIntakeChallenge[currentCommand].getValue() > 0){
          if(ahrs.getYaw() < autonomousIntakeChallenge[currentCommand].getValue()){
            moveMotors(0.3, -0.3);
          } else {
            currentCommand++;
            ahrs.zeroYaw();
            FrontR.setSelectedSensorPosition(0.0);
          }
        } else {
         if(ahrs.getYaw() > autonomousIntakeChallenge[currentCommand].getValue()){
            moveMotors(-0.3, 0.3);
         } else {
            currentCommand++;
            ahrs.zeroYaw();
            FrontR.setSelectedSensorPosition(0.0);
          }
        }
      }
    } else {
      moveMotors(0, 0);
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    FrontL.setSelectedSensorPosition(0.0);
    mytimer.reset();
    mytimer.start();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if(mytimer.get() > 0.5){
      System.out.println("Distance Travelled: " + unitToDistance(FrontR.getSelectedSensorPosition()));
      mytimer.reset();
      mytimer.start();
    }
    //SHOOTER -------------------------------------------------------------------------
    //R1 to shoot
    //Y to stop
    if (stick.getRawButton(6)){
      shooter.set(-0.70);
    }
    if(stick.getRawButton(4)){
      shooter.set(0);
    }

    //Using the D-PAD to control speed of shooter
    if(stick.getPOV() == 0){
      shooterSpeed = -0.80;
    } else if(stick.getPOV() == 90){
      shooterSpeed = -0.70;
    } else if(stick.getPOV() == 180){
      shooterSpeed = -0.80;
    } else if(stick.getPOV() == 270){
      shooterSpeed = -0.85;
    }

    //ELEVATOR ------------------------------------------------------------------------
    //L1 to Elevator
    if (stick.getRawButton(5)) {
      elevator.set(ControlMode.PercentOutput, -0.55);
    } else {
      elevator.set(ControlMode.PercentOutput, 0);
    }

    //INTAKE --------------------------------------------------------------------------
    //A to intake
    //B to reverse intake if Ball is stuck
    if (stick.getRawButton(1)) {
      intake.set(ControlMode.PercentOutput, 0.40);
    } else if (stick.getRawButton(2)) {
      intake.set(ControlMode.PercentOutput, -0.40);
    } else {
      intake.set(ControlMode.PercentOutput, 0);
    }

    //DRIVING CODE --------------------------------------------------------------------
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
    
    FrontL.set(ControlMode.PercentOutput, maxSpeedCheck(w[0] / (wMax*1.7 )));
    FrontR.set(ControlMode.PercentOutput, -maxSpeedCheck(w[1] / (wMax*1.7 )));
    BackL.set(ControlMode.PercentOutput, maxSpeedCheck(w[2] / (wMax*1.7 )));
    BackR.set(ControlMode.PercentOutput, -maxSpeedCheck(w[3] / (wMax*1.7 )));
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

  public double unitToDistance(double pos){
    return (pos/4096) * (6.0 * Math.PI);
  }

  public void moveMotors(double left, double right){
    FrontL.set(ControlMode.PercentOutput, left);
    BackL.set(ControlMode.PercentOutput, left);
    FrontR.set(ControlMode.PercentOutput, -1 * right);
    BackR.set(ControlMode.PercentOutput, -1 * right);
  }
}