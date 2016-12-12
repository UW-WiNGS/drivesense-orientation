package main;

import gpsevaluation.GPSAbstraction;
import io.DirectoryWalker;
import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.List;

import sensors.CoordinateAlignment;
import sensors.OrientationChangeDetection;
import tracereplay.RealTimeBehaviorDetector;
import utility.Constants;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class XSense {

	public static void start() {
		accuracyEvaluation();
	}
	
	
	private static void accuracyEvaluation() {
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);	
		List<Trace> output = new ArrayList<Trace>();

		for(String directory: folders) {
			String names[] = directory.split("/");
			String highway = directory.concat("/highway");
			String urban = directory.concat("/urban");
				
			List<Trip> htrips = ReadWriteTrace.loadTrips(highway);	
			List<Trip> trips = ReadWriteTrace.loadTrips(urban);
			trips.addAll(htrips);
			for(Trip trip: trips) {
				List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
				if(accelerometer.size() < 600 || OrientationChangeDetection.orientationChanged(accelerometer)) {
					continue;
				}
					
				List<Trace> cur = alignAndCompare(trip);
				if(cur == null) continue;
				output.addAll(cur);
				
				if(output.size() >= 50000) break;
			}
			//ReadWriteTrace.writeFile(output, outpath.concat(names[names.length - 1] + ".dat"));
		}
		ReadWriteTrace.writeFile(output, Constants.kAlterSenseOutput.concat("evaluation/accuracy/acceleration.dat"));

	}
	
	
	private static List<Trace> compareSteering(Trip trip) {
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = PreProcess.exponentialMovingAverage(trip.rotation_matrix_, -1);
		List<Trace> speed = CoordinateAlignment.calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 1.0));		
		List<Trace> gps = PreProcess.exponentialMovingAverage(GPSAbstraction.wrapperGPS(trip.gps_elevation_), 3);
		
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		CoordinateAlignment.trainDetector(detector, trip);
		List<Trace> projected_accelerometer = CoordinateAlignment.alignAccelerometer(trip, detector);
		if(projected_accelerometer == null) {
			return null;
		}
		CoordinateAlignment.setStop(projected_accelerometer);

		
		List<Trace> res = new ArrayList<Trace>();
		
		
		return res;
		
	}
	
	private static List<Trace> alignAndCompare(Trip trip) {
		List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = PreProcess.exponentialMovingAverage(trip.rotation_matrix_, -1);
		List<Trace> speed = CoordinateAlignment.calculateAccelerationByOBD(PreProcess.interpolate(trip.speed_, 1.0));		
		List<Trace> gps = PreProcess.exponentialMovingAverage(GPSAbstraction.wrapperGPS(trip.gps_elevation_), 3);
		
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		CoordinateAlignment.trainDetector(detector, trip);
		List<Trace> projected_accelerometer = CoordinateAlignment.alignAccelerometer(trip, detector);
		if(projected_accelerometer == null) {
			return null;
		}
		CoordinateAlignment.setStop(projected_accelerometer);
		
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0; i < speed.size(); ++i) {
			Trace curspeed = speed.get(i);
			double speedacce = curspeed.values[1];
			Trace curgps = PreProcess.getTraceAt(gps, curspeed.time);
			double gpsacce = 0.0;
			if(curgps == null) {
				gpsacce = speedacce * 2;
			} else {
				gpsacce = curgps.values[3];
			}
			Trace cursensor = PreProcess.getTraceAt(projected_accelerometer, curspeed.time);
			if(cursensor == null) {
				continue;
			}
			double sensoracce = cursensor.values[1];
			
			Trace ntr = new Trace(3);
			ntr.time = trip.time_;
			ntr.values[0] = Math.abs(gpsacce - speedacce);
			ntr.values[1] = Math.abs(sensoracce - speedacce);
			ntr.values[2] = Math.min(ntr.values[0], ntr.values[1]);
			
			if(ntr.values[2] > 3.0) {
				return null;
			}
			res.add(ntr);
		}
		return res;
	}
	
}
