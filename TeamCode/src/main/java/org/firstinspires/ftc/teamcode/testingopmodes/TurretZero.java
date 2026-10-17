package org.firstinspires.ftc.teamcode.testingopmodes;

import static org.firstinspires.ftc.teamcode.subsystems.Turret.PULLEY_RATIO;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.TURRET_ZERO_DEG;
import static org.firstinspires.ftc.teamcode.subsystems.Turret.voltageToDegrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Point the turret straight forward by hand, then copy "New TURRET_ZERO_DEG" into Turret.java.
 */
@TeleOp(name = "Turret Zero", group = "Robot")
public class TurretZero extends LinearOpMode {
    @Override
    public void runOpMode() {
        AnalogInput externalEncoder = hardwareMap.get(AnalogInput.class, "externalEncoder");

        while (!isStopRequested()) {
            double encoderDeg = voltageToDegrees(externalEncoder.getVoltage());
            double zero = encoderDeg * PULLEY_RATIO;

            telemetry.addLine("Point the turret straight forward by hand");
            telemetry.addData("Encoder Voltage", "%.3f", externalEncoder.getVoltage());
            telemetry.addData("Turret Degrees (current zero)", "%.1f", zero - TURRET_ZERO_DEG);
            telemetry.addData("New TURRET_ZERO_DEG", "%.1f", zero);
            telemetry.update();
        }
    }
}
