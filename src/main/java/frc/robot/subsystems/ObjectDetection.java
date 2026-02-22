package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase{
    // Name of our camera
    PhotonCamera camera = new PhotonCamera("photonvision");   
    
    // Query the latest result from PhotonVision
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();    

    // Check if the latest result has any targets.
    boolean hasTargets = (results.size() > 0);
    

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults();
    }

    @Override
    public void simulationPeriodic() {
        results = camera.getAllUnreadResults();
        Logger.recordOutput("ObjectDetection/PipelineHasTargets", hasTargets);
    }

}
