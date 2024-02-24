package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Climber;

public class RightClimbCommandIn extends Command{

    private final Climber mClimber;

    public RightClimbCommandIn(Climber climber){
        mClimber = climber;
    }
    
    @Override
    public void execute(){
        mClimber.RightClimbCommandIn();
    }
    @Override
    public void end(boolean interrupted){
        mClimber.NoClimbCommand();
    }

}
