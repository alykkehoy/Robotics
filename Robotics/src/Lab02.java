import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.sensor.NXTUltrasonicSensor;


public class Lab02 {
	
	private static NXTUltrasonicSensor ultrasensor = new NXTUltrasonicSensor(SensorPort.S2);
	
	public static void main(String[] args) {
		/* NXTLightSensor s1 = new NXTLightSensor(SensorPort.S1);
		NXTLightSensor s2 = new NXTLightSensor(SensorPort.S2); */
		
		
		final SampleProvider test = ultrasensor.getDistanceMode();
		int distanceValue = 0;
		
		final int iteration_threshold = 100;
		for(int i = 0; i <= iteration_threshold; i++){
			float [] sample3 = new float[test.sampleSize()];
			test.fetchSample(sample3, 0);
			distanceValue = (int)sample3[0];
			
			System.out.println(sample3[0]);
			
			Delay.msDelay(500);
			
		}
	}
}