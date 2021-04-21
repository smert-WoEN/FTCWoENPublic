package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RollingAverage
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.highH
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.highS
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.highV
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.lowH
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.lowS
import org.firstinspires.ftc.teamcode.robot.OpenCVNodeWebcam.OpenCVConfig.lowV
import org.firstinspires.ftc.teamcode.superclasses.RobotModule
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

open class OpenCVNodeWebcam : RobotModule() {

    @Config
    internal object OpenCVConfig {
        @JvmField var lowH = 7.0 //13
        @JvmField var lowS = 130.0 //110
        @JvmField var lowV = 130.0 //110
        @JvmField var highH = 22.0 //20
        @JvmField var highS = 255.0
        @JvmField var highV = 255.0
    }

    lateinit var webcam: OpenCvCamera
    override fun initialize() {
        try {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1"),
                                                                    opMode.hardwareMap.appContext.resources.getIdentifier(
                                                                         "cameraMonitorViewId", "id",
                                                                         opMode.hardwareMap.appContext.packageName))
            webcam.setPipeline(pipeline)
            webcam.openCameraDeviceAsync {
                webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT)
            }
        } catch (e: Exception) {
            opMode.telemetry.addData("OpenCVNode Error", e.message)
        }
    }


    fun stopCam() {
        val finalStackSize = stackSize
        try {
            webcam.closeCameraDeviceAsync { stackSize = finalStackSize }
        } catch (ignored: Exception) {
        }
    }

    fun retrieveResult(): StackSize {
        val stackSize = stackSize
        stopCam()
        return stackSize
    }

    @Volatile var stackSize = StackSize.ZERO
    @Volatile var mean = 0.0
    @Volatile var aspectRatio = 0.0

    enum class StackSize {
        ZERO, ONE, FOUR
    }

    val pipeline = object : OpenCvPipeline() {
        private val averageResult = RollingAverage(10)
        private val ZERO_RINGS = 0
        private val ONE_RING = 1
        private val FOUR_RINGS = 2
        private val stages = Stage.values()
        private var all = Mat()
        var ringStackBoundingRect = Rect()
        var hsvLowerBound = Scalar(lowH, lowS, lowV)
        var hsvUpperBound = Scalar(highH, highS, highV)
        private var wbReferenceScalar = Scalar(128.0, 128.0, 128.0)
        private var wbReferenceLum = .0
        private var HSVMat = Mat()
        private var HSVMatMean = Scalar(.0, .0, .0)
        private var thresholdMat = Mat()
        private val structuringElement = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, Size(cols / 14.0, cols / 48.0))
        private val crop = Rect(0, cols - (cols * 2) / 3, rows, (cols * 2) / 3)
        private var wbReferenceRect = Rect(rows / 2 - (rows / 2) / 2, cols - cols / 4, rows / 2, cols / 4) //cols/3
        private val BlurSize = Size(cols / 50.0, cols / 50.0)
        private var stageToRenderToViewport = Stage.DETECTION
        private val autoTapper = ElapsedTime()
        override fun onViewportTapped() {
            val currentStageNum = stageToRenderToViewport.ordinal
            var nextStageNum = currentStageNum + 1
            if (nextStageNum >= stages.size) {
                nextStageNum = 0
            }
            stageToRenderToViewport = stages[nextStageNum]
        }

        override fun processFrame(input: Mat): Mat {
            try {
                wbReferenceScalar = Core.mean(input.submat(wbReferenceRect))
                wbReferenceLum = (wbReferenceScalar.`val`[0] + wbReferenceScalar.`val`[1] + wbReferenceScalar.`val`[2]) / 3.0
                all = input.submat(crop)
                if (wbReferenceLum != 0.0) {
                    wbReferenceScalar = Scalar(wbReferenceLum / wbReferenceScalar.`val`[0],
                                               wbReferenceLum / wbReferenceScalar.`val`[1],
                                               wbReferenceLum / wbReferenceScalar.`val`[2])
                    Core.multiply(all, wbReferenceScalar, all)
                }
                Imgproc.GaussianBlur(all, all, BlurSize, 0.0)
                Imgproc.cvtColor(all, HSVMat, Imgproc.COLOR_RGB2HSV)
                hsvLowerBound = Scalar(lowH, lowS, lowV)
                hsvUpperBound = Scalar(highH, highS, highV)
                HSVMatMean = Core.mean(HSVMat)
                Core.inRange(HSVMat, Scalar(hsvLowerBound.`val`[0], (hsvLowerBound.`val`[1] + HSVMatMean.`val`[1]) / 2.0,
                                            (hsvLowerBound.`val`[2] + HSVMatMean.`val`[2]) / 2.0), hsvUpperBound, thresholdMat)
                Imgproc.erode(thresholdMat, thresholdMat, structuringElement)
                Imgproc.dilate(thresholdMat, thresholdMat, structuringElement)
                ringStackBoundingRect = Imgproc.boundingRect(thresholdMat)
                Imgproc.rectangle(input, crop, Scalar(0.0, 255.0, 0.0), 2)
                Imgproc.rectangle(input, wbReferenceRect, Scalar(128.0, 128.0, 128.0), 2)
                Imgproc.rectangle(all, ringStackBoundingRect, Scalar(0.0, 0.0, 255.0), 2)
                mean = Core.mean(thresholdMat).`val`[0]
                if (mean > 0.1) {
                    aspectRatio = ringStackBoundingRect.width.toDouble() / ringStackBoundingRect.height.toDouble()
                    if (aspectRatio > 2.2) averageResult.addNumber(ONE_RING) else averageResult.addNumber(FOUR_RINGS)
                } else {
                    averageResult.addNumber(ZERO_RINGS)
                    aspectRatio = 0.0
                }
                stackSize = when (averageResult.average) {
                     ONE_RING -> StackSize.ONE
                     FOUR_RINGS -> StackSize.FOUR
                    else -> StackSize.ZERO
                }

                if (autoTapper.seconds() > 3) {
                    onViewportTapped()
                    autoTapper.reset()
                }
                return when (stageToRenderToViewport) {
                     Stage.DETECTION -> {
                          all
                     }
                     Stage.RAW_IMAGE -> {
                          input.submat(crop)
                     }
                     Stage.THRESHOLD -> {
                          thresholdMat
                     }
                }
            } catch (e: Exception) {
                stackSize = StackSize.ZERO
                return Mat()
            }
        }

    }

    enum class Stage {
        DETECTION,  //includes outlines
        THRESHOLD,  //b&w
        RAW_IMAGE //displays raw view
    }

    companion object {
        const val rows = 640
        const val cols = 480
    }
}