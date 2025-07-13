package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

public class OtherLoggers {
	private static boolean enabled = false;
	private static final CANStatus status = new CANStatus();
	private static PowerDistribution pdh;

	private static Notifier radioNotifier;
	private static Notifier mainNotifier;

	public static void start(LoggedRobot robot, PowerDistribution pdh) {
		SmartDashboard.putData("PDH",pdh);

		if (enabled) return;
		enabled = true;
		OtherLoggers.pdh = pdh;


        mainNotifier = new Notifier(() -> {
            try {
                logSystem();
                logCan();
                logPdh();
            } catch (Exception e) {
                DriverStation.reportError("OtherLoggers exception: " + e, true);
            }
        });
        mainNotifier.startPeriodic(0.04); //25 hz

	}

	private static void logSystem() {
		Logger.recordOutput("System/SerialNumber", HALUtil.getSerialNumber());
		Logger.recordOutput("System/Comments", HALUtil.getComments());
		Logger.recordOutput("System/Active", HAL.getSystemActive());
		Logger.recordOutput("System/BrownedOut", HAL.getBrownedOut());
		Logger.recordOutput("System/RSLState", HAL.getRSLState());
		Logger.recordOutput("System/SystemTimeValid", HAL.getSystemTimeValid());
		Logger.recordOutput("System/BrownoutVoltage", PowerJNI.getBrownoutVoltage());
		Logger.recordOutput("System/CPUTempC", PowerJNI.getCPUTemp());

		Logger.recordOutput("Power/BatteryVoltage", PowerJNI.getVinVoltage());
		Logger.recordOutput("Power/BatteryCurrent", PowerJNI.getVinCurrent());

		Logger.recordOutput("3V3/Voltage", PowerJNI.getUserVoltage3V3());
		Logger.recordOutput("3V3/Current", PowerJNI.getUserCurrent3V3());
		Logger.recordOutput("3V3/Active", PowerJNI.getUserActive3V3());
		Logger.recordOutput("3V3/Faults", PowerJNI.getUserCurrentFaults3V3());

		Logger.recordOutput("5V/Voltage", PowerJNI.getUserVoltage5V());
		Logger.recordOutput("5V/Current", PowerJNI.getUserCurrent5V());
		Logger.recordOutput("5V/Active", PowerJNI.getUserActive5V());
		Logger.recordOutput("5V/Faults", PowerJNI.getUserCurrentFaults5V());

		Logger.recordOutput("6V/Voltage", PowerJNI.getUserVoltage6V());
		Logger.recordOutput("6V/Current", PowerJNI.getUserCurrent6V());
		Logger.recordOutput("6V/Active", PowerJNI.getUserActive6V());
		Logger.recordOutput("6V/Faults", PowerJNI.getUserCurrentFaults6V());
	}

	private static void logCan() {
		CANJNI.getCANStatus(status);
		Logger.recordOutput("CAN/Utilization", status.percentBusUtilization);
		Logger.recordOutput("CAN/BusOffCount", status.busOffCount);
		Logger.recordOutput("CAN/TxFullCount", status.txFullCount);
		Logger.recordOutput("CAN/RxErrorCount", status.receiveErrorCount);
		Logger.recordOutput("CAN/TxErrorCount", status.transmitErrorCount);
	}

	private static void logPdh() {
		if (pdh == null) return;
		Logger.recordOutput("PDH/Temperature", pdh.getTemperature());
		Logger.recordOutput("PDH/Voltage", pdh.getVoltage());
		Logger.recordOutput("PDH/ChannelCurrents", pdh.getAllCurrents());
		Logger.recordOutput("PDH/TotalCurrent", pdh.getTotalCurrent());
		Logger.recordOutput("PDH/TotalPower", pdh.getTotalPower());
		Logger.recordOutput("PDH/TotalEnergy", pdh.getTotalEnergy());
		Logger.recordOutput("PDH/ChannelCount", pdh.getNumChannels());
	}



}
