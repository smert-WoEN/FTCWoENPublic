package org.firstinspires.ftc.teamcode.robot.legacy

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector

@Deprecated("")
class TFdetector(opMode: LinearOpMode?) : Runnable {
    private var recognitionResult = 0
    var opMode: LinearOpMode? = null
    private var uptime = ElapsedTime()
    private lateinit var vuforia: VuforiaLocalizer
    private lateinit var tfod: TFObjectDetector
    private var doStop = false
    fun retrieveResult(): Int {
        doStop()
        return recognitionResult
    }

    @Synchronized
    fun doStop() {
        doStop = true
    }

    @Synchronized
    private fun keepRunning(): Boolean {
        return !doStop
    }

    override fun run() {
        doStop = false
        uptime.reset()
        while (keepRunning() && !if (opMode == null) opMode!!.isStopRequested else opMode!!.isStopRequested) {
            val updatedRecognitions = tfod.updatedRecognitions
            if (updatedRecognitions != null) {
                recognitionResult = 0
                for (recognition in updatedRecognitions) {
                    recognitionResult = try {
                        recognition.label.toInt()
                    } catch (e: NumberFormatException) {
                        0
                    }
                }
            }
            opMode!!.telemetry.addData("Recognition result: ", recognitionResult)
            opMode!!.telemetry.update()
        }
        tfod.shutdown()
    }

    fun initialize() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        val parameters = VuforiaLocalizer.Parameters()
        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        /*
         * Initialize the TensorFlow Object Detection engine.
         */
        val tfodParameters = TFObjectDetector.Parameters(
            opMode!!.hardwareMap.appContext.resources.getIdentifier(
                "tfodMonitorViewId", "id", opMode!!.hardwareMap.appContext.packageName
            )
        )
        tfodParameters.minResultConfidence = 0.5f
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia)
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT)
        tfod.activate()
    }

    companion object {
        private const val TFOD_MODEL_ASSET = "UltimateGoal.tflite"
        private const val LABEL_FIRST_ELEMENT = "4"
        private const val LABEL_SECOND_ELEMENT = "1"
        private const val VUFORIA_KEY =
            "AYQyDib/////AAABmWoLWPk9RUbvpT7hIVtMz+KJ7Wgtz7khQCon2wk+3/Mt+oIFV0pwc6vrhxOD2hI8Vh9IvPuTzPC2zBiOYGLIrg9m4lskp19GIKC6mv4bGqkZC0aLiJWnW5SSZRC5inIVhz+PxiQVYqhTVUskF9/ab2xuAFxohYL2mqdxuZPGyqLvpqEwuWWKiecF3S2fkKeQ+3yyryRMQhSd648Tl1NzaRWWXsUDStrFLfCAp+K922bBJaquOpraQ6aP1vu/oPlu7fbxxAcJytVPX81ASdjyPd4gDPp/tYEPk/xs7avDKYvdnBUM/RKxmIVkiWtFuiA5ug2DHM3mPfxm0peM8+2kQVjbGQLRUJdKKmp/QBjCfVOp"
    }

    init {
        this.opMode = opMode
    }
}