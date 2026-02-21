package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetection extends SubsystemBase{
    // Name of our camera
    PhotonCamera camera = new PhotonCamera("photonvision");   
    
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult();    

    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();

    @Override
    public void periodic() {
        
    }

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput("ObjectDetection/PipelineHasTargets", hasTargets);
    }

}
