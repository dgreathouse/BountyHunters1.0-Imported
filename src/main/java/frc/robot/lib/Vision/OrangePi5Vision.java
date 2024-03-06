// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class OrangePi5Vision {
    PhotonCamera camera;
    PhotonCamera camera2;
    PhotonPipelineResult results;
    public static double m_yaw;
    public OrangePi5Vision(){
       camera = new PhotonCamera("NexiGo");
       
       //camera = new PhotonCamera("photonvision");
        setDriverMode(false);
        setPipelineIndex(0);
    }
    public PhotonPipelineResult getResults(){
        return camera.getLatestResult();
    }
    public boolean hasTargets(){
        return getResults().hasTargets();
    }
    /**
     * 
     * @return The Yaw in degrees with Positive right
     */
    private double getCameraNoteYaw(){
        results = camera.getLatestResult();

        if(results.hasTargets()){
            return -results.getBestTarget().getYaw();
        }else {
            return 180.0;
        }
    }
    public void setPipelineIndex(int _x){
        camera.setPipelineIndex(_x);
    }
    public void setDriverMode(boolean _mode){
        camera.setDriverMode(_mode);
    }
    public double getNoteYaw(){
        return m_yaw;
    }
    public void findNote(){
        m_yaw = getCameraNoteYaw();
    }
}
