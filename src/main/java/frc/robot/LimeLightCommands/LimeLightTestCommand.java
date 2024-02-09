package frc.robot.LimeLightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.LimeLight;

public class LimeLightTestCommand extends Command{
    private final LimeLight mLimelight;
    public LimeLightTestCommand(LimeLight limelight){
        mLimelight = limelight;
    }

    @Override
    public void execute(){
        mLimelight.LimeLightTestCommand();
    }


    
}
