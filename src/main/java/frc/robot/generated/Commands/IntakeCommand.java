package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Intake;

public class IntakeCommand extends Command{
    
    private final Intake mIntake;


    public IntakeCommand(Intake intake){

    mIntake = intake;
    }

@Override
public void execute(){
   mIntake.IntakeCommand();
}
@Override
public void end(boolean interrupted){
    mIntake.StopIntakingCommand();
}

}
