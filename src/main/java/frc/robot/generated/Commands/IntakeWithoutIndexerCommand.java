package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Intake;

public class IntakeWithoutIndexerCommand extends Command{
    
    private final Intake mIntake;


    public IntakeWithoutIndexerCommand(Intake intake){

    mIntake = intake;
    }

@Override
public void execute(){
   mIntake.IntakeWithoutIndexer();
}
@Override
public void end(boolean interrupted){
    mIntake.StopIntakingCommand();
}

}
