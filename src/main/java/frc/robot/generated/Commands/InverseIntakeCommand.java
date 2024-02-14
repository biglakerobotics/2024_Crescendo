package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Intake;

public class InverseIntakeCommand extends Command{
    
    private final Intake mIntake;


    public InverseIntakeCommand(Intake intake){

    mIntake = intake;
    }

@Override
public void execute(){
   mIntake.InverseIntakeCommand();
}
@Override
public void end(boolean interrupted){
    mIntake.StopIntakingCommand();
}

}
