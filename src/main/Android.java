package main;

import io.DirectoryWalker;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.List;

import sensors.SensorCluster;
import tracereplay.RealTimeSensorProcessing;
import tracereplay.TraceReplayEngine;
import utility.Constants;
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
	}

}
