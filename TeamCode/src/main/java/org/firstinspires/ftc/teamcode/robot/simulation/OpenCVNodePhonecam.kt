package org.firstinspires.ftc.teamcode.robot.simulation

import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvInternalCamera

class OpenCVNodePhonecam : OpenCVNodeWebcam() {
    override fun initialize() {
        try {
            webcam = OpenCvCameraFactory.getInstance().createInternalCamera(
                OpenCvInternalCamera.CameraDirection.BACK,
                opMode.hardwareMap.appContext.resources.getIdentifier(
                    "cameraMonitorViewId",
                    "id",
                    opMode.hardwareMap.appContext.packageName
                )
            ) as OpenCvInternalCamera
            webcam.setPipeline(pipeline)
            webcam.openCameraDeviceAsync {
                webcam.startStreaming(
                    rows,
                    cols,
                    OpenCvCameraRotation.SIDEWAYS_LEFT
                )
            }
        } catch (e: Exception) {
            opMode.telemetry.addData("OpenCVNode Error", e.message)
        }
    }
}