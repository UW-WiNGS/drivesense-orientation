package gpsevaluation;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import utility.Formulas;
import utility.Pattern;
import utility.PreProcess;
import utility.Trace;

public class LanechangeExtraction {
	
	

	/*a really small turn threshold*/
	private static double kLaneChangeThreshold = 0.17;
	private static long kMinimumDuration = 2000;
	private static int kSlidingWindowSize = 20;
	private static int kZIndex = 2;


	
	private static boolean isLanechange(List<Trace> gyroscope, Pattern pattern) {
		
		if(pattern.end - pattern.start < kMinimumDuration) return false;
		

		List<Trace> sub = (List<Trace>) PreProcess.extractSubList(gyroscope, pattern.start, pattern.end);
		int sz = sub.size();
		double rads = 0.0;
		
		for(int i = 0; i < sz - 1; ++i) {
			Trace trace = sub.get(i);
			long time_diff = gyroscope.get(i + 1).time - gyroscope.get(i).time;			
			double z = trace.values[kZIndex];
			rads += z * (time_diff/1000.0);
		}
		//Log.log(pattern.start - start*1000, pattern.end - start*1000, Math.toDegrees(rads));
		if(Math.abs(Math.toDegrees(rads)) < 10.0)
			return true;
		return false;
	}
	
	
	static public List<Pattern> extractLanechanges(List<Trace> gyroscope) {
		
		///List<Trace> div = new ArrayList<Trace>();
		
		int wnd = kSlidingWindowSize;
		List<Pattern> patterns = new ArrayList<Pattern>();
		int sz = gyroscope.size();
		//Log.log(Thread.currentThread().getStackTrace()[1].getMethodName(), "the size of input traces is:" + String.valueOf(sz));
		LinkedList<Trace> sliding = new LinkedList<Trace>();		
		boolean in_turn = false;
		//int d = gyroscope.get(sz - 1).dim;
		Pattern new_pattern = null;
		for(int i = 0; i < sz; ++i) {
			Trace trace = gyroscope.get(i);
			sliding.add(trace);
			int len = sliding.size();
			if(len==wnd) {
				double [] deviation = Formulas.absoluteDeviation(sliding);
				boolean turnning  = false;	
				
				if(deviation[kZIndex] > kLaneChangeThreshold)
					turnning = true;
				if(turnning) {
					/*static*/
					if(false==in_turn) {
						in_turn = true;
						new_pattern = new Pattern();
						new_pattern.start = gyroscope.get(i - wnd + 1).time;
					}
				} else {
					if(true==in_turn) {
						in_turn = false;
						new_pattern.end = gyroscope.get(i - 1).time;
						if(isLanechange(gyroscope, new_pattern)) {
							new_pattern.start = new_pattern.start - 1000;
							new_pattern.end = new_pattern.end + 1000;
							new_pattern.type = Pattern.kLanechange;
							patterns.add(new_pattern);
						}
						new_pattern = null;
					}
				}
				sliding.removeFirst();
			}
		}
		if(null!=new_pattern && new_pattern.end == - 1) {
			new_pattern.end = gyroscope.get(sz - 1).time;
			if(isLanechange(gyroscope, new_pattern)) {
				new_pattern.start = new_pattern.start - 1000;
				new_pattern.end = new_pattern.end + 1000;
				new_pattern.type = Pattern.kLanechange;
				patterns.add(new_pattern);
			}
			new_pattern = null;
		}
		return patterns;
	}
	
	
	
	
	private static int kXIndex = 0;
	private static double kLaneChangeThresholdByAccelerometer = 2;
	private static boolean isLanechangeByAccelerometer(List<Trace> accelerometer, Pattern pattern) {
		
		if(pattern.end - pattern.start < kMinimumDuration) return false;
		List<Trace> sub = (List<Trace>) PreProcess.extractSubList(accelerometer, pattern.start, pattern.end);
		int sz = sub.size();
		double sum = 0.0;
		
		for(int i = 0; i < sz - 1; ++i) {
			Trace trace = sub.get(i);
			sum += trace.values[0];
		}
		//Log.log(sum/sz);
		if(Math.abs(sum/sz) < 0.2)
			return true;
		return false;
	}
	/**
	 * Extract lane changes by accelerometer
	 * @param accelerometer
	 * @return
	 */
	
	static public List<Pattern> extractLanechangesByAccelerometer(List<Trace> accelerometer) {

		int wnd = kSlidingWindowSize;
		List<Pattern> patterns = new ArrayList<Pattern>();
		int sz = accelerometer.size();
		LinkedList<Trace> sliding = new LinkedList<Trace>();		
		boolean in_turn = false;
		Pattern new_pattern = null;
		for(int i = 0; i < sz; ++i) {
			Trace trace = accelerometer.get(i);
			sliding.add(trace);
			int len = sliding.size();
			if(len==wnd) {
				double [] deviation = Formulas.absoluteDeviation(sliding);
				boolean turnning  = false;	
				//Log.log(deviation[0]);
				if(deviation[kXIndex] > kLaneChangeThresholdByAccelerometer)
					turnning = true;
				if(turnning) {
					/*static*/
					if(false==in_turn) {
						in_turn = true;
						new_pattern = new Pattern();
						new_pattern.start = accelerometer.get(i - wnd + 1).time;
					}
				} else {
					if(true==in_turn) {
						in_turn = false;
						new_pattern.end = accelerometer.get(i - 1).time;
						if(isLanechangeByAccelerometer(accelerometer, new_pattern)) {
							new_pattern.start = new_pattern.start - 1000;
							new_pattern.end = new_pattern.end + 1000;
							new_pattern.type = Pattern.kLanechange;
							patterns.add(new_pattern);
						}
						new_pattern = null;
					}
				}
				sliding.removeFirst();
			}
		}
		if(null!=new_pattern && new_pattern.end == - 1) {
			new_pattern.end = accelerometer.get(sz - 1).time;
			if(isLanechangeByAccelerometer(accelerometer, new_pattern)) {
				new_pattern.start = new_pattern.start - 1000;
				new_pattern.end = new_pattern.end + 1000;
				new_pattern.type = Pattern.kLanechange;
				patterns.add(new_pattern);
			}
			new_pattern = null;
		}
		return patterns;
	}

}
