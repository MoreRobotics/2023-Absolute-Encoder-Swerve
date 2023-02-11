/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Limelight extends SubsystemBase {

  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;
  public Pose2d prevEstimatedRobotPose;

  public Limelight() {
    // Set up a test arena of two apriltags at the center of each driver station set
    //  final AprilTag tag18 =
    //          new AprilTag(
    //                  18,
    //                  new Pose3d(
    //                          new Pose2d(
    //                                  FieldConstants.length,
    //                                  FieldConstants.width / 2.0,
    //                                  Rotation2d.fromDegrees(180))));
    //  final AprilTag tag01 =
    //          new AprilTag(
    //                  01,
    //                  new Pose3d(new Pose2d(0.0, FieldConstants.width / 2.0, Rotation2d.fromDegrees(0.0))));
    //  ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
    //  atList.add(tag18);
    //  atList.add(tag01);

    AprilTagFieldLayout aprilTagFieldLayout = loadFieldLayout();

    // Forward Camera
    photonCamera = new PhotonCamera(VisionConstants.cameraName); // Change the name of your camera here to whatever it is in the
    // PhotonVision UI.

    // Create pose estimator
    // TODO: Change robotToCam to match where the limelight is
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);
  }
 
  /**
  * @param estimatedRobotPose The current best guess at robot pose
  * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
  *     of the observation. Assumes a planar field and the robot is always firmly on the ground
  */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public Optional<EstimatedRobotPose> setFirstEstimatedGlobalPose() {
    Pose2d zero = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
    photonPoseEstimator.setReferencePose(zero);
    return photonPoseEstimator.update();
  }

  private AprilTagFieldLayout loadFieldLayout() {
    try {
      return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError("Unable to load April Tag Field Layout", true);
      e.printStackTrace();
      // return an empty AprilTagFieldLayout
      return new AprilTagFieldLayout(new ArrayList<AprilTag>(), FieldConstants.length, FieldConstants.width);
    }
  }

     
 }