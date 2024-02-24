package frc.robot.generated;



import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.Networking.LimeLightTable;
import frc.robot.LimelightHelpers;
import frc.robot.LimeLightCommands.LimeLightTestCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight{

    private static LimeLight instance;


    private LimeLightTable m_LimeLightTable;

    private double tx;


    public void LimeLightTestCommand(){

        double[] botpose = LimelightHelpers.getBotPose("limelight-spud");

        double ty = LimelightHelpers.getTY("limelight-spud");
        double ta = LimelightHelpers.getTA("limelight-spud");

        tx = LimelightHelpers.getTX("limelight-spud");

        if (tx>0) {
            System.out.println("Turn Right");
            System.out.println(tx);
            System.out.println("Botpose:");
            System.out.println(botpose);
            System.out.println("TY");
            System.out.println(ty);
            System.out.println("TA");
            System.out.println(ta);
            
            
        }else{
            System.out.println("Turn Left");
            System.out.println(tx);
            System.out.println("Botpose:");
            System.out.println(botpose);
            System.out.println("TY");
            System.out.println(ty);
            System.out.println("TA");
            System.out.println(ta);
        }


    
    }

    public LimeLight(){
        m_LimeLightTable = LimeLightTable.getInstance();
    }


    public double getXdeviation() {
        return m_LimeLightTable.getTx();
    }

    public double getYdeviation() {
        return m_LimeLightTable.getTy();
    }

    public double[] getTranslationToAprilTag(){
        return m_LimeLightTable.getTranslationToAprilTag();
    }

    public void enableVisionProcessing(){
        m_LimeLightTable.setCamMode(0);
        m_LimeLightTable.setPipeline(1);
        m_LimeLightTable.setLedMode(3);
        System.out.println("Vision processing enabled");
    }

    public void enableDriverCamera(){
        m_LimeLightTable.setCamMode(1);
        m_LimeLightTable.setLedMode(1);
    }

    public double getCamMode() {
        return m_LimeLightTable.getCamMode();
    }

    public void toggleLED() {
        if (m_LimeLightTable.getLedMode() == 1){
            m_LimeLightTable.setLedMode(3);
        }else{
            m_LimeLightTable.setLedMode(1);
        }
    }

    public Boolean isVisionProcessing() {
        return m_LimeLightTable.getCamMode() == 0 && m_LimeLightTable.getLedMode() == 3;
    }
    public double getTargetID() {
        return m_LimeLightTable.getCurrentApriltagId();
    }

    public static LimeLight getInstance(){
        if(instance == null){
            instance = new LimeLight();
        }
        return instance;
    }

}

