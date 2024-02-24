package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Intake;

public class IntakeWithIndexerCommand extends Command{
    
    // public DigitalInput limitSwitch = new DigitalInput(0);
    
    private final Intake mIntake;

    private boolean mpiecein = false;

    public IntakeWithIndexerCommand(Intake intake){

    mIntake = intake;
    }

@Override
public void initialize(){
    // mpiecein = !limitSwitch.get();
    // if(!mpiecein){
        // mIntake.IntakeCommand();
    // }
}

@Override
public void execute(){
    // if(!limitSwitch.get()){
    // System.out.println("Loaded");
    //     mIntake.StopIntakingCommand();
    //     mpiecein = true;
    // System.out.println("NewLineV");
    // }
    mpiecein = mIntake.IntakeCommand();
    if(mpiecein) {
        mIntake.StopIntakingCommand();
    }
}
@Override
public void end(boolean interrupted){
        mIntake.StopIntakingCommand();
}

@Override
public boolean isFinished(){
    mIntake.StopIntakingCommand();
    return mpiecein;
}

}
