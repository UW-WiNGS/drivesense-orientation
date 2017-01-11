package main;

import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import tracereplay.RealTimeSensorProcessing;
import tracereplay.TraceReplayEngine;
import utility.Constants;
import utility.EventTrace;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class Android {
	
	public static void start() {
		
		List<String> suman = DirectoryWalker.getFilePaths(Constants.kUncontrol.concat("suman/"));
		
		List<String> files = suman;
		
		
		RealTimeSensorProcessing detector = new RealTimeSensorProcessing();
		
		List<Trace> res = new ArrayList<Trace>();
		for(String file: files) {
			String name = file.substring(file.length() - 16, file.length() - 3);
			long time = Long.valueOf(name);
			Trip trip = SqliteAccess.loadTrip(file, time);
			
			test(detector, trip);
			//int changes = detectOrientationChange(accelerometer);
			//double var = SensorCluster.calculateClusterVariance(accelerometer.subList(0, 30*20));
			
			
			
			break;
		}
	}
	
	
	
	private static double kTurnThreshold = 0.05;
	private static int kWindowSize = 15;
	private static void extractTurn(List<Trace> proj_gyro) {
		int counter = 0;
		double turnsum = 0.0;
		EventTrace event = null;
		List<Trace> window_gyroscope = new LinkedList<Trace>();
		List<Trace> turns = new ArrayList<Trace>();
		
		List<EventTrace> events = new ArrayList<EventTrace>();
		for (int i = 0; i < proj_gyro.size(); i++) {
			Trace trace = proj_gyro.get(i);
			double value = trace.values[2];
			
			Trace past = null;
			window_gyroscope.add(trace);
			if(window_gyroscope.size() >= 15) {
				past = window_gyroscope.remove(0);
			} else {
				continue;
			}
			Trace pre = window_gyroscope.get(15 - 2);
			
			if(Math.abs(value) >= kTurnThreshold)  counter++;			
			double pv = past.values[2];			
			if(Math.abs(pv) >= kTurnThreshold)	counter--;
			

			boolean turning = false;
			if((double)counter >= (double)kWindowSize * 0.6) {
				turning = true;
			}
			
			Trace ntr = new Trace(1);
			ntr.time = trace.time;
			if(turning) {
				ntr.values[0] = 1;
				turnsum += pre.values[2] * (trace.time - pre.time)/1000.0;
				if(event == null) {
					event = new EventTrace();
					event.start_ = trace.time;
				}
			} else {
				ntr.values[0] = 0;
				if(event != null) {
					event.end_ = trace.time;
					event.type_ = event.TURN;
					if(Math.toDegrees(turnsum) >= 60.0){
						Log.d("recording turn");
						events.add(event);
					}
				}
				turnsum = 0.0;
				event = null;
			}
			turns.add(ntr);
		}
		ReadWriteTrace.writeFile(turns, "/home/lkang/projects/sensorgps/android/".concat("turns.dat"));
		
		for(EventTrace ent: events) {
			Log.d("Android", ent.start_, ent.end_);
		}
	}
	
	public static void test(RealTimeSensorProcessing detector, Trip trip) {
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = PreProcess.exponentialMovingAverage(trip.rotation_matrix_, -1);
		List<Trace> gps = trip.gps_;
				
		/** 
		 * feed the data into the real time detector
		 */
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		input.add(gps);
		TraceReplayEngine.traceReplay(input, detector);
		
		
		
		
		///////////////////////////////////////////////////////////////
		List<Trace> projected_gyroscope = new ArrayList<Trace>();
		for(int i = 0; i < gyroscope.size(); ++i) {
			Trace cur = gyroscope.get(i);
			Trace projected = Formulas.rotate(cur, detector.getInitRM().values);
			
		
			projected_gyroscope.add(projected);
		}
		ReadWriteTrace.writeFile(projected_gyroscope, "/home/lkang/projects/sensorgps/android/".concat("projected_gyroscope.dat"));
		
		extractTurn(projected_gyroscope);
		
		
		
	}
	

}
