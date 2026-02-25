from typing import List
from constants import AutoConstants, VisionConstants

from simple_state_system import *

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Pose2d, Pose3d

from photonlibpy import PhotonCamera, PhotonPoseEstimator, EstimatedRobotPose

from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult


class VisionSubsystem(StateSystem):
    photon_camera: PhotonCamera
    pose_estimator: PhotonPoseEstimator
    robot_pose: EstimatedRobotPose | None

    april_tag_field_layout: AprilTagFieldLayout
    best_april_tag_pose: Pose2d

    def __init__(self, camera_name: str) -> None:
        # Initialize the state machine
        super().__init__()

        try:
            self.april_tag_field_layout = AprilTagFieldLayout.loadField(
                AprilTagField.kDefaultField
            )
        except RuntimeError as e:
            raise e

        self.photon_camera = PhotonCamera(camera_name)

        self.pose_estimator = PhotonPoseEstimator(
            self.april_tag_field_layout,
            VisionConstants.robot_to_camera,
        )

        self.robot_pose = EstimatedRobotPose(
            Pose3d(),
            0,
            [],
        )

    def periodic(self) -> None:
        # Run internal periodic functions
        super().periodic()

        print("Hello, world!")

        camera_results: List[PhotonPipelineResult] = (
            self.photon_camera.getAllUnreadResults()
        )

        if len(camera_results) == 0:
            return

        for result in camera_results:
            if not result.hasTargets():
                continue

            potential_tag_pose: Pose3d | None = self.april_tag_field_layout.getTagPose(
                result.getBestTarget().getFiducialId()
            )

            if not potential_tag_pose == None:
                self.best_april_tag_pose = potential_tag_pose.toPose2d()

            self.robot_pose = self.pose_estimator.update(result)
