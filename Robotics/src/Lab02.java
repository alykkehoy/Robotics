import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.sensor.NXTUltrasonicSensor;

public class Lab02 {
	
	private static NXTUltrasonicSensor ultrasonic_front = new NXTUltrasonicSensor(SensorPort.S2);
	private static NXTUltrasonicSensor ultrasonic_side = new NXTUltrasonicSensor(SensorPort.S1);
	
	public static void main(String[] args) {
		EV3LargeRegulatedMotor motor_r = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor motor_l = new EV3LargeRegulatedMotor(MotorPort.B);
		motor_r.forward();
		motor_l.forward();
		
		final SampleProvider test_front = ultrasonic_front.getDistanceMode();
		final SampleProvider test_side = ultrasonic_side.getDistanceMode();

		float [] sample_front = new float[test_front.sampleSize()];
		float [] sample_side = new float[test_side.sampleSize()];
		
		float distance_front = 0;
		float distance_side = 0;

		while(true){
			test_front.fetchSample(sample_front, 0);
			test_side.fetchSample(sample_side, 0);

			float error_front = distance_front - sample_front[0];
			float error_side = distance_side - sample_side[0];

			motor_r.setSpeed(0);
			motor_l.setSpeed(0);
		}
	}
	
	public float calc_p(){
		float p_val = 0;
		return p_val;
	}
	
	public float calc_i(){
		float i_val = 0;
		return i_val;
	}
	
	public float calc_d(){
		float d_val = 0;
		return d_val;
	}
}