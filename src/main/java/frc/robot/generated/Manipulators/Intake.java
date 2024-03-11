package frc.robot.generated.Manipulators;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.generated.Commands.IntakeWithoutIndexerCommand;
import frc.robot.generated.TunerConstants.IntakeConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Intake extends TimedRobot{
    //  public AnalogInput ultrasonicSensor = new AnalogInput(0);
    public DigitalInput limitSwitch = new DigitalInput(0);
    private boolean mpiecein = false;

    private final TalonFX IntakeMotor = new TalonFX(22);
    private final TalonFX IntakeMotor2 = new TalonFX(23);

    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    double currentDistanceCm = 0.0;



    ShuffleboardTab tab = Shuffleboard.getTab("CompTab");

    
    public boolean IntakeCommand(){
        if(!limitSwitch.get()){
            System.out.println("Loaded");
                StopIntakingCommand();
                mpiecein = true;
            System.out.println("NewLineV");
        } else {
            IntakeMotor.set(.20);
            IntakeMotor2.set(.20);
            mpiecein = false;
        }
        Shuffleboard.getTab("CompTab").add("PieceIn",mpiecein);
        return mpiecein;
        
    }

    public void IntakeWithoutIndexer(){
        IntakeMotor.set(IntakeConstants.INTAKESPEED);
        IntakeMotor2.set(IntakeConstants.INTAKESPEED);
    }

    public void InverseIntakeCommand(){
        IntakeMotor.set(-IntakeConstants.INTAKESPEED);
        IntakeMotor2.set(-IntakeConstants.INTAKESPEED);
    }


    public void StopIntakingCommand(){
        IntakeMotor.set(0);
        IntakeMotor2.set(0);

    }
}