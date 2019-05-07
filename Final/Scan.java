import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Scan extends Thread{
	
	private static NXTUltrasonicSensor ultrasonic_left = new NXTUltrasonicSensor(SensorPort.S1);
	private static NXTUltrasonicSensor ultrasonic_right = new NXTUltrasonicSensor(SensorPort.S2);
	
	private static EV3LargeRegulatedMotor sensormotor_r = new EV3LargeRegulatedMotor(MotorPort.B);
	private static EV3LargeRegulatedMotor sensormotor_l = new EV3LargeRegulatedMotor(MotorPort.C);
	
	SampleProvider test_left;
	SampleProvider test_right;
	
	float[] left_sample;
	float[] right_sample;

	public Scan(float[] left_sample, float[] right_sample){
		this.left_sample = left_sample;
		this.right_sample = right_sample;
	}
	
	@Override 
	public void run(){
//		this.ultrasonic_left = ultrasonic_left;
//		this.ultrasonic_right = ultrasonic_right;
		
//		this.test_left = ultrasonic_left.getDistanceMode();
//		this.test_right = ultrasonic_right.getDistanceMode();
		
		int left_turn = 0;
		int right_turn = 0;
		
		while(true){
			Motor.B.rotate(30);
			right_turn += 30;
			Motor.C.rotate(-30);
			left_turn -= 30;
			
			
			switch(left_turn){
			case 0:
				ultrasonic_left.fetchSample(left_sample, 0);
				break;
			case -30:
				ultrasonic_left.fetchSample(left_sample, 1);
				break;
			case -60:
				ultrasonic_left.fetchSample(left_sample, 2);
				Motor.B.rotate(60);
				left_turn += 60;
				break;
			}
			
			
			switch(right_turn){
			case 0:
				ultrasonic_right.fetchSample(right_sample, 0);
				break;
			case 30:
				ultrasonic_right.fetchSample(right_sample, 1);
				break;
			case 60:
				ultrasonic_right.fetchSample(right_sample, 2);
				Motor.B.rotate(-60);
				left_turn -= 60;
				break;
			}

		}
//		this.sample_left = new float[test_left.sampleSize()];
//		this.sample_right = new float[test_right.sampleSize()];		
	}
}
