#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>


class VisionSubsystem {

    public:

        VisionSubsystem();
        std::optional<std::pair<frc::Pose2d, units::second_t>> GetLatestPose();

    private:

    const frc::Transform3d kRobotToCamera{5.5_in, -11.625_in, 24.0_in, frc::Rotation3d{units::degree_t{5.0}, units::degree_t{0.0}, units::degree_t{180.0}}};

    photon::PhotonCamera m_camera{"launcher"};
    photon::PhotonPoseEstimator m_poseEstimator{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded), kRobotToCamera};
    photon::PhotonPipelineResult m_latestResult;

};