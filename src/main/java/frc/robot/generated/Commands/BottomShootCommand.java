package frc.robot.generated.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.Manipulators.BottomShooter;

public class BottomShootCommand extends Command{

    private final BottomShooter mShooter;

    public BottomShootCommand(BottomShooter shooter){
        mShooter = shooter;
    }
    
    @Override
    public void execute(){
        mShooter.ShootCommand();
    }
    @Override
    public void end(boolean interrupted){
        mShooter.StopShootingCommand();
    }

}
