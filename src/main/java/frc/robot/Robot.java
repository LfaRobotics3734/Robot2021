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
  private TalonSRX elevator, intake, belt; 
  private TalonSRX FrontL, FrontR, BackL, BackR;
  private VictorSP shooter;
  private double shooterSpeed = -0.65;
  private AHRS ahrs;
  private double distance;
  private String orientation;
  private boolean shooting = false;
  //if turn:
  // + deg is right
  // - deg is left
  private int radius = 21;
  private AutonomousCommands[] autonomousIntakeChallenge = new AutonomousCommands[]{
    /*new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", -90),
    new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", 87),
    new AutonomousCommands("move", .0),
    new AutonomousCommands("turn", 87),
    new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", -90),
    new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", -90),
    new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", -90),
    new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", -90),
    new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", 87),
    new AutonomousCommands("move", 180.0),
    new AutonomousCommands("turn", 87),
    new AutonomousCommands("move", 60.0),
    new AutonomousCommands("turn", -90),
    new AutonomousCommands("move", 60.0),*/
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
    belt = new TalonSRX(6);

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
        orientation=event.value.getValue().toString();
    }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    ahrs.zeroYaw();
    FrontR.setSelectedSensorPosition(0.0);
    mytimer.start();
    currentCommand = 0;
    while(orientation==null){
    }
    //Path B
    if(orientation.equals("left")){ 
      //blue path
      autonomousIntakeChallenge=new AutonomousCommands[]{
        new AutonomousCommands("turn", -90.0)
      };
    }
    else if(orientation.equals("right")){ 
      //red path
      autonomousIntakeChallenge=new AutonomousCommands[]{
        new AutonomousCommands("turn", 90.0)
      };
    }
    else{
      //Path A
      if(Math.abs(distance-140)<10){
        //Blue Path
        autonomousIntakeChallenge=new AutonomousCommands[]{
          new AutonomousCommands("move", 150+radius),
          new AutonomousCommands("turn", -71.6),
          new AutonomousCommands("move", 94.9+radius),
          new AutonomousCommands("turn", 98.2),
          new AutonomousCommands("move", 67.1*2),
          
        };
      }
      else{
        //Red Path
        autonomousIntakeChallenge=new AutonomousCommands[]{
          new AutonomousCommands("move", 60 +radius),
          new AutonomousCommands("turn", 22.5),
          new AutonomousCommands("move", 67.2+radius),
          new AutonomousCommands("turn", -(71.6+22.5)),
          new AutonomousCommands("move", 91.8+radius),
          new AutonomousCommands("turn", 71.6),
          new AutonomousCommands("move", 150)
          
        };
      }
    }
      
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*
  intake.set(ControlMode.PercentOutput, 0.4);
  System.out.println(unitToDistance(FrontR.getSelectedSensorPosition()));
   if(currentCommand < autonomousIntakeChallenge.length){
      if(autonomousIntakeChallenge[currentCommand].getCommandType() == "move"){
        if(unitToDistance(FrontR.getSelectedSensorPosition()) < autonomousIntakeChallenge[currentCommand].getValue()){
          //accelerate(0.5, mytimer);
          moveMotors(0.5, 0.5);
        } else {
          currentCommand++;
          FrontR.setSelectedSensorPosition(0.0);
          mytimer.reset();
          mytimer.start();
          ahrs.zeroYaw();
        }
      } else {
        if(autonomousIntakeChallenge[currentCommand].getValue() > 0){
          if(ahrs.getYaw() < autonomousIntakeChallenge[currentCommand].getValue()){
            moveMotors(0.2, -0.2);
          } else {
            currentCommand++;
            ahrs.zeroYaw();
            FrontR.setSelectedSensorPosition(0.0);
            mytimer.reset();
            mytimer.start();
          }
        } else {
         if(ahrs.getYaw() > autonomousIntakeChallenge[currentCommand].getValue()){
            moveMotors(-0.2, 0.2);
         } else {
            currentCommand++;
            ahrs.zeroYaw();
            FrontR.setSelectedSensorPosition(0.0);
            mytimer.reset();
            mytimer.start();
          }
        }
      }
    } else {
      moveMotors(0, 0);
    } 
    */
    if(unitToDistance(FrontR.getSelectedSensorPosition()) < 60.0){
      moveMotors(0.4,0.4);
    } else {
      moveMotors(0,0);
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    FrontL.setSelectedSensorPosition(0.0);
    shooting = false;
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    //SHOOTER -------------------------------------------------------------------------
    //Y to start and stop shooter
    if(stick.getRawButton(4)){
        shooter.set(0);
    }
    if(stick.getRawButton(6)){
        shooter.set(shooterSpeed);
    }

    //Using the D-PAD to control speed of shooter
    if(stick.getPOV() == 0){
      shooterSpeed = -0.65;
    } else if(stick.getPOV() == 90){
      shooterSpeed = -0.70;
    } else if(stick.getPOV() == 180){
      shooterSpeed = -0.80;
    } else if(stick.getPOV() == 270){
      shooterSpeed = -0.82;
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
      belt.set(ControlMode.PercentOutput, 0.35);
    } else if (stick.getRawButton(2)) {
      intake.set(ControlMode.PercentOutput, -0.40);
      belt.set(ControlMode.PercentOutput, -0.35);
    } else {
      intake.set(ControlMode.PercentOutput, 0);
      belt.set(ControlMode.PercentOutput, 0);
    }

    //DRIVING CODE --------------------------------------------------------------------
    
    if(stick.getRawAxis(2) > 0.3 || stick.getRawAxis(3) > 0.3){
      if(stick.getRawAxis(3) > 0.3){
        //right
        FrontR.set(ControlMode.PercentOutput,0.3);
        BackR.set(ControlMode.PercentOutput,-0.3);
        BackL.set(ControlMode.PercentOutput,-0.3);
        FrontL.set(ControlMode.PercentOutput,0.3);
      } else if(stick.getRawAxis(2) > 0.3){
        //left strafe
        FrontR.set(ControlMode.PercentOutput,-0.3);
        BackR.set(ControlMode.PercentOutput,0.3);
        BackL.set(ControlMode.PercentOutput,0.3);
        FrontL.set(ControlMode.PercentOutput,-0.3);
      } else {
        moveMotors(0,0);
      }
    } else {

      if(stick.getRawAxis(1) < 0.2 && stick.getRawAxis(1) > -0.2){
        if(stick.getRawAxis(4) < 0.2 && stick.getRawAxis(4) > -0.2){
          moveMotors(0,0);
        } else {
          moveMotors(stick.getRawAxis(4) * 0.5, -stick.getRawAxis(4) * 0.5);
        }
      } else {
        double speed = stick.getRawAxis(1) * -0.6;
        if(stick.getRawAxis(4) < 0.2 && stick.getRawAxis(4) > -0.2){
          moveMotors(speed, speed);
        } else {
          if(stick.getRawAxis(4) > 0){
            //right
            double newSpeed = speed - (speed * stick.getRawAxis(4));
            moveMotors(speed, newSpeed);
          } else {
            //left
            double newSpeed = speed - (speed * Math.abs(stick.getRawAxis(4)));
            moveMotors(newSpeed, speed);
          }
        }
      }  
    }


  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    FrontR.set(ControlMode.PercentOutput, 0.2);
  }

  public double unitToDistance(double pos){
    return (pos/4096) * (6.0 * Math.PI);
  }
  public void accelerate(double speed, Timer timer){
    double time = 0.5;
    if(timer.get() < time){
      moveMotors((timer.get()/time)*speed,(timer.get()/time)*speed);
    }
    else{
      moveMotors(speed,speed);
    }
  }

  public void moveMotors(double left, double right){
    FrontL.set(ControlMode.PercentOutput, left);
    BackL.set(ControlMode.PercentOutput, left);
    FrontR.set(ControlMode.PercentOutput, -1 * right);
    BackR.set(ControlMode.PercentOutput, -1 * right);
  }
}
