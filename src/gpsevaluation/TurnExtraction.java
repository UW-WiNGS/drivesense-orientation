package gpsevaluation;

import java.util.ArrayList;
import java.util.List;

import utility.Pattern;
import utility.PreProcess;
import utility.Trace;


/**
 * 
 * 
 */

public class TurnExtraction {
	//sliding windows size
	
	/*a really small turn threshold*/
	private static double kTurnThreshold = 0.08;
	private static int kSlidingWindowSize = 20;
	private static int kZIndex = 2;
	private static double kPercent = 0.7;
	

	private static boolean isTurn(List<Trace> gyroscope, Pattern pattern) {
		
		if(pattern.end - pattern.start < 2000) return false;
		
		List<Trace> sub = (List<Trace>) PreProcess.extractSubList(gyroscope, pattern.start, pattern.end);
		int sz = sub.size();
		double rads = 0.0;
		
		for(int i = 0; i < sz - 1; ++i) {
			Trace trace = sub.get(i);
			long time_diff = gyroscope.get(i + 1).time - gyroscope.get(i).time;			
			double z = trace.values[kZIndex];
			rads += z * (time_diff/1000.0);
		}
		//Log.log(rads, Math.toDegrees(rads));
		if(Math.abs(Math.toDegrees(rads)) > 60.0) {
			pattern.accumulated_change = Math.toDegrees(rads);
			return true;
		}
		return false;
	}
	
	public static List<Pattern> extractTurns(List<Trace> gyroscope) {
		List<Pattern> patterns = new ArrayList<Pattern>();	
		int sz = gyroscope.size();
		int counter = 0;
		boolean inTurn  = false;
		
		Pattern new_pattern = null;
		for (int i = 0; i < sz; i++) {
			Trace trace = gyroscope.get(i);
			double value = trace.values[kZIndex];
			if(Math.abs(value) >= kTurnThreshold)  counter++;
			if (i < kSlidingWindowSize) continue;
			
			/*when i >= window size, we track the first item in the window size*/
			Trace past = gyroscope.get(i - kSlidingWindowSize);	
			double pv = past.values[kZIndex];			
			if(Math.abs(pv) >= kTurnThreshold)	counter--;
			

			boolean turning = false;
			if((double)counter/(double)kSlidingWindowSize > kPercent) {
				turning = true;
			}
			
			if(turning) {
				if(!inTurn) {
					inTurn = true;
					new_pattern = new Pattern();
					new_pattern.start = gyroscope.get(i - kSlidingWindowSize + 1).time;
				}
			} else {
				if(inTurn) {
					inTurn = false;
					new_pattern.end = gyroscope.get(i - 1).time;
					new_pattern.type = Pattern.kTurn;
					if(isTurn(gyroscope, new_pattern)) {
						patterns.add(new_pattern);
					}
					new_pattern = null;
				}
			}
		}
		if(null!=new_pattern) {
			new_pattern.end = gyroscope.get(sz - 1).time;
			new_pattern.type = Pattern.kTurn;
			if(isTurn(gyroscope, new_pattern)) {
				patterns.add(new_pattern);
			}
		}
		return patterns;
	}
	
	
	private static double kAccelerometerTurnThreshold = 0.5;
	private static int kXIndex = 0;
	private static double kAccelerometerPercent = 0.5;
	
	/**
	 * Extract the turns by accelerometer if gyroscope is not available on device
	 * @param accelerometer
	 * @return the extracted patterns
	 */
	public static List<Pattern> extractTurnsByAccelerometer(List<Trace> accelerometer) {
		List<Pattern> patterns = new ArrayList<Pattern>();
		int sz = accelerometer.size();
		int counter = 0;
		boolean inTurn  = false;
		
		Pattern new_pattern = null;
		for (int i = 0; i < sz; i++) {
			Trace trace = accelerometer.get(i);
			double value = trace.values[kXIndex];
			if(Math.abs(value) >= kAccelerometerTurnThreshold)  counter++;
			if (i < kSlidingWindowSize) continue;
			
			/*when i >= window size, we track the first item in the window size*/
			Trace past = accelerometer.get(i - kSlidingWindowSize);	
			double pv = past.values[kXIndex];			
			if(Math.abs(pv) >= kAccelerometerTurnThreshold)	counter--;
			

			boolean turning = false;
			if((double)counter/(double)kSlidingWindowSize > kAccelerometerPercent) {
				turning = true;
			}
			
			if(turning) {
				if(!inTurn) {
					inTurn = true;
					new_pattern = new Pattern();
					new_pattern.start = accelerometer.get(i - kSlidingWindowSize + 1).time;
				}
			} else {
				if(inTurn) {
					inTurn = false;
					new_pattern.end = accelerometer.get(i - 1).time;
					new_pattern.type = Pattern.kTurn;
					if(new_pattern.end - new_pattern.start >= 2000) {
						patterns.add(new_pattern);
					}
					new_pattern = null;
				}
			}
		}
		if(null!=new_pattern) {
			new_pattern.end = accelerometer.get(sz - 1).time;
			new_pattern.type = Pattern.kTurn;
			if(new_pattern.end - new_pattern.start >= 2000) {
				patterns.add(new_pattern);
			}
		}
		return patterns;
	}

}
