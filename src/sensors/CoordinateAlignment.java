package sensors;

import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import tracereplay.RealTimeBehaviorDetector;
import tracereplay.TraceReplayEngine;
import utility.Constants;
import utility.Formulas;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class CoordinateAlignment {
	
	private static final String input = Constants.datPath.concat("lei/urban/1397761356431/");
	private static final String output = Constants.outputPath.concat("alignment/data/");
	
	public static void start() {
		Trip trip = ReadWriteTrace.loadTrip(input);
		offlineAlignment(trip);
	}
	
	
	private static Trace alignmentPencentileAccuracy(List<Trace> errors) {
		
		Collections.sort(errors, new Comparator<Trace>() {
			public int compare(Trace tr0, Trace tr1) {
				if (tr0.values[3] < tr1.values[3])
					return -1;
				else if (tr0.values[3] > tr1.values[3])
					return 1;
				else
					return 0;
	        }
	    });
		int num = 10;
		int sz = errors.size();
		if(sz < num) {
			return null;
		}
		
		Trace res = new Trace(num);
		for(int i = 0; i < num; ++i) {
			res.values[i] = errors.get(sz/num * (i + 1) - 1).values[3];
		}
		return res;
	}
	
	private static List<Trace> alignmentError(List<Trace> speed, List<Trace> projected_accelerometer) {
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0; i < speed.size(); ++i) {
			Trace cur = speed.get(i);
			Trace corres = PreProcess.getTraceAt(projected_accelerometer, cur.time);
			if(corres == null) {
				continue;
			}
			Trace ntr = new Trace(4);
			ntr.time = cur.time;
			ntr.values[0] = cur.values[1]; // the acceleration from speed
			ntr.values[1] = corres.values[1];
			ntr.values[2] = corres.values[1] - cur.values[1];
			ntr.values[3] = Math.abs(corres.values[1] - cur.values[1]);
			if(Math.abs(cur.values[1]) >= 0.1) {
				res.add(ntr);
			}
		}
		
		return res;
	}
	
	public static Trace alignmentPerformance(Trip trip) {
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = PreProcess.exponentialMovingAverage(trip.rotation_matrix_, -1);
		List<Trace> speed = calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 2.0));		
		/** 
		 * feed the data into the real time detector
		 */
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		TraceReplayEngine.traceReplay(input, detector);
		
		Trace rm = detector.getInitRM();
		Trace hrm = detector.getHorizontalRM();
		if(rm == null || hrm == null) {
			return null;
		}
		int reverse_counter = 0;
		List<Trace> projected_accelerometer = new ArrayList<Trace>();
		for(Trace trace: accelerometer) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace hrot = Formulas.rotate(initrot, hrm.values);
			
			Trace ntr = new Trace(3);
			ntr.copyTrace(hrot);
			projected_accelerometer.add(ntr);
			
			long time = ntr.time;
			Trace curspeed = PreProcess.getTraceAt(speed, time);
			if(curspeed != null) {
				double correct = ntr.values[1] - curspeed.values[1];
				double reverse = ntr.values[1] + curspeed.values[1];
				if(Math.abs(reverse) < Math.abs(correct)) {
					reverse_counter ++;
				}
			}
		}
		if(reverse_counter >= 0.3 * projected_accelerometer.size()) {
			for(Trace trace: projected_accelerometer) {
				trace.values[0] = -trace.values[0];
				trace.values[1] = -trace.values[1];
			}
		}
		
		List<Trace> errors = alignmentError(speed, projected_accelerometer);
		if(errors == null) return null;
		Trace percentile = alignmentPencentileAccuracy(errors);
		return percentile;
	}
	
	private static List<Trace> offlineAlignment(Trip trip) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = PreProcess.exponentialMovingAverage(trip.rotation_matrix_, -1);
		List<Trace> speed = calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 2.0));
		
		
		/** 
		 * feed the data into the real time detector
		 */
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		TraceReplayEngine.traceReplay(input, detector);
				
		
		Trace rm = detector.getInitRM();
		Trace hrm = detector.getHorizontalRM();

		List<Trace> projected_accelerometer = new ArrayList<Trace>();
		for(Trace trace: accelerometer) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace hrot = Formulas.rotate(initrot, hrm.values);
			Trace ntr = new Trace(3);
			ntr.copyTrace(hrot);
			projected_accelerometer.add(ntr);
		}
		ReadWriteTrace.writeFile(projected_accelerometer, output.concat("/projected_accelerometer.dat"));
		ReadWriteTrace.writeFile(speed, output.concat("/speed.dat"));
		
		List<Trace> accuracy = alignmentError(speed, projected_accelerometer);
		ReadWriteTrace.writeFile(accuracy, output.concat("/accuracy.dat"));
		
		
		List<Trace> projected_gyroscope = new ArrayList<Trace>();
		for(Trace trace: gyroscope) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace hrot = Formulas.rotate(initrot, hrm.values);
			Trace ntr = new Trace(3);
			ntr.copyTrace(hrot);
			projected_gyroscope.add(ntr);
		}
		ReadWriteTrace.writeFile(projected_gyroscope, output.concat("/projected_gyroscope.dat"));
		
		
		
		return res;
	}
	
	private static List<Trace> calculateAccelerationByOBD(List<Trace> speed) {
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0; i < speed.size() - 1; ++i) {
			Trace cur = speed.get(i);
			Trace next = speed.get(i + 1);
			Trace ntr = new Trace(2);
			ntr.time = cur.time;
			ntr.values[0] = cur.values[0] * Constants.kKmPHToMeterPS;
			ntr.values[1] = (next.values[0] - cur.values[0]) * Constants.kKmPHToMeterPS / ((next.time - cur.time)/1000.0);
			res.add(ntr);
		}
		return res;
	}

}
