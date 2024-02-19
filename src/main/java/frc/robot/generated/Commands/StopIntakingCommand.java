package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Intake;

public class  StopIntakingCommand extends Command{
    
    private final Intake mIntake;


    public StopIntakingCommand(Intake intake){

    mIntake = intake;
    }

@Override
public void execute(){
   mIntake.StopIntakingCommand();
}
@Override
public void end(boolean interrupted){
    mIntake.StopIntakingCommand();
}

}
