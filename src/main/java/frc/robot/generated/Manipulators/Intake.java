package frc.robot.generated.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.generated.TunerConstants.IntakeConstants;

public class Intake {
    private final TalonFX IntakeMotor = new TalonFX(2);
    private final TalonFX IntakeMotor2 = new TalonFX(3);



    public void IntakeCommand(){
IntakeMotor.set(IntakeConstants.INTAKESPEED);
IntakeMotor2.set(IntakeConstants.INTAKESPEED);
    }


public void StopIntakingCommand(){
    IntakeMotor.set(0);
    IntakeMotor2.set(0);

}
}