package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.VoltageSupplier

class VoltageSupplier: MultithreadedRobotModule(), VoltageSupplier{
    private val defaultVoltage = 12.0
    override var voltage = defaultVoltage //average
    var averageVoltage = defaultVoltage
    private val pauseTime = 100.0
    private val voltageTime = ElapsedTime()
    private val nSamples = 5

    lateinit var voltageSensor: VoltageSensor

    override fun initialize() {
        voltageSensor = WoENHardware.controlHubVoltageSensor
    }

    override fun updateControlHub() {

        if(voltageTime.milliseconds() > pauseTime) {
            averageVoltage -= averageVoltage / nSamples
            averageVoltage += voltageSensor.voltage / nSamples
            voltage = averageVoltage
            voltageTime.reset()
            //TODO Rolling average
            //TODO Query timeout
        }

    }
}