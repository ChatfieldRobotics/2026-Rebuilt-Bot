from typing import List
from constants import AutoConstants, VisionConstants

from simple_state_system import *

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

from wpimath.geometry import Pose2d, Pose3d

from photonlibpy import PhotonCamera, PhotonPoseEstimator, EstimatedRobotPose

from photonlibpy.photonPoseEstimator import PhotonPoseEstimator
from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult


class VisionSubsystem(StateSystem):
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

        self.robot_pose: EstimatedRobotPose = EstimatedRobotPose(
            Pose3d(),
            0,
            [],
        )

    def periodic(self):
        # Run internal periodic functions
        super().periodic()

        if not hasattr(self, "photon_camera"):
            return

        camera_results: List[PhotonPipelineResult] = (
            self.photon_camera.getAllUnreadResults()
        )

        if len(camera_results) == 0:
            self.robot_pose = None
            return

        for result in camera_results:
            if not result.hasTargets():
                continue

            potential_tag_pose: Pose3d | None = self.april_tag_field_layout.getTagPose(
                result.getBestTarget().getFiducialId()
            )

            if not potential_tag_pose == None:
                self.best_april_tag_pose = potential_tag_pose.toPose2d()

            self.robot_pose = self.pose_estimator.estimateCoprocMultiTagPose(result)

            if self.robot_pose is None:
                self.robot_pose = self.pose_estimator.estimateLowestAmbiguityPose(result)