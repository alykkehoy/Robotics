import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Scan extends Thread{
	
	private static NXTUltrasonicSensor ultrasonic_left = new NXTUltrasonicSensor(SensorPort.S1);
	private static NXTUltrasonicSensor ultrasonic_right = new NXTUltrasonicSensor(SensorPort.S2);
		
//	SampleProvider test_left;
//	SampleProvider test_right;
	
	final static int MAX_RANGE = 100;
	
	float[] left_sample;
	float[] right_sample;

	public Scan(float[] left_sample, float[] right_sample){
		this.left_sample = left_sample;
		this.right_sample = right_sample;
	}
	
	@Override 
	public void run(){		
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
				if(left_sample[0] > MAX_RANGE){
					left_sample[0] = 0;
				}
				break;
			case -30:
				ultrasonic_left.fetchSample(left_sample, 1);
				if(left_sample[1] > MAX_RANGE){
					left_sample[1] = 0;
				}
				break;
			case -60:
				ultrasonic_left.fetchSample(left_sample, 2);
				if(left_sample[2] > MAX_RANGE){
					left_sample[2] = 0;
				}
				Motor.B.rotate(60);
				left_turn += 60;
				break;
			}
			
			
			switch(right_turn){
			case 0:
				ultrasonic_right.fetchSample(right_sample, 0);
				if(right_sample[0] > MAX_RANGE){
					right_sample[0] = 0;
				}
				break;
			case 30:
				ultrasonic_right.fetchSample(right_sample, 1);
				if(right_sample[1] > MAX_RANGE){
					right_sample[1] = 0;
				}
				break;
			case 60:
				ultrasonic_right.fetchSample(right_sample, 2);
				if(right_sample[2] > MAX_RANGE){
					right_sample[2] = 0;
				}
				Motor.B.rotate(-60);
				left_turn -= 60;
				break;
			}
		}
	}
}
