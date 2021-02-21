package frc.robot;

public class AutonomousCommands {
    private String commandType;
    private double value;

    public AutonomousCommands(String command, double value){
        this.commandType = command;
        this.value = value;
    }

    public String getCommandType(){
        return this.commandType;
    }

    public double getValue(){
        return this.value;
    }
}
