package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.Climber;

public class RightClimbCommandOut extends Command{

    private final Climber mClimber;

    public RightClimbCommandOut(Climber climber){
        mClimber = climber;
    }
    
    @Override
    public void execute(){
        mClimber.RightClimbCommandOut();
    }
    @Override
    public void end(boolean interrupted){
        mClimber.NoClimbCommand();
    }

}
