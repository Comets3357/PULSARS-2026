#include "Subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem() {

}


std::optional<std::pair<frc::Pose2d, units::second_t>> VisionSubsystem::GetLatestPose(){
    photon::PhotonPipelineResult result = m_camera.GetLatestResult();
    if (result == m_latestResult){
        return std::nullopt;
    }
    m_latestResult = result;
    
    std::optional<photon::EstimatedRobotPose> pose = m_poseEstimator.EstimateCoprocMultiTagPose(result);

    if (!pose.has_value()){
        pose = m_poseEstimator.EstimateLowestAmbiguityPose(result);
    }

    if (pose.has_value()){
        return std::optional<std::pair<frc::Pose2d, units::second_t>>{{pose.value().estimatedPose.ToPose2d(), pose.value().timestamp}};
    }
    
    return std::nullopt;
}


