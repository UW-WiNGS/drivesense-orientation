package main;

import io.DirectoryWalker;
import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.Log;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class GPSEvaluation {
	
	private static String TAG = "GPSEvaluation";
	
	public static void start() {
		String outfolder = Constants.outputPath.concat("gpsevaluation/compareobdgps/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		double sum = 0.0;
		List<Trace> output = new ArrayList<Trace>();
		for(String directory: folders) {
			//Log.log(TAG, directory);
			String urban = directory.concat("/urban");
			String highway = directory.concat("/highway");
			List<Trip> trips = ReadWriteTrace.loadTrips(urban);
			for(Trip trip: trips) {
				//Trace cur = checkMissingGPS(trip);
				//output.add(cur);
				//List<Trace> cur = compareGPSAndOBDSpeed(trip);
				List<Trace> cur = compareGPSAndOBDAcceleration(trip);
				output.addAll(cur);
			}
			//break;
		}
		//Log.log(sum);
		ReadWriteTrace.writeFile(output, outfolder.concat("urban.dat"));
		
	}
	


	private static List<Trace> compareGPSAndOBDAcceleration(Trip trip) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> speed = PreProcess.interpolate(trip.speed_, 2.0);
		List<Trace> gps = trip.gps_elevation_;
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace curgps = gps.get(i);
			Trace nextgps = gps.get(i + 1);
			double gpsacce = (nextgps.values[3] - curgps.values[3])/((nextgps.time - curgps.time)/1000.0);
			
			Trace curobd = PreProcess.getTraceAt(speed, curgps.time);
			Trace nextobd = PreProcess.getTraceAt(speed, nextgps.time);
			if(curobd == null || nextobd == null) continue;
			double obdacce = (nextobd.values[0] - curobd.values[0]) * Constants.kKmPHToMeterPS/((nextobd.time - curobd.time)/1000.0);
			
			Trace cur = new Trace(3);
			cur.time = curgps.time;
			cur.values[0] = gpsacce;
			cur.values[1] = obdacce;
			res.add(cur);
		}
		
		return res;
	}
	

	private static List<Trace> compareGPSAndOBDSpeed(Trip trip) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> speed = PreProcess.interpolate(trip.speed_, 2.0);
		List<Trace> gps = trip.gps_elevation_;
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace curgps = gps.get(i);
			Trace curobd = PreProcess.getTraceAt(speed, curgps.time);
			if(curobd == null) continue;
			Trace cur = new Trace(3);
			cur.time = curgps.time;
			cur.values[0] = curgps.values[3] - curobd.values[0] * Constants.kKmPHToMeterPS;
			res.add(cur);
		}
		
		return res;
	}
	
	private static Trace checkMissingGPS(Trip trip) {
		List<Trace> speed = trip.speed_;
		List<Trace> gps = trip.gps_elevation_;
		List<Trace> accelerometer = trip.accelerometer_;
		
		//Log.log(speed.size(), gps.size(), accelerometer.size());
		//Log.log(trip.path);
		double dist = 0.0;
		for(int i = 0; i < speed.size() - 1; ++i) {
			Trace cur = speed.get(i);
			Trace next = speed.get(i + 1);
			long diff = next.time - cur.time;
			double spd = cur.values[0];
			dist += (spd * Constants.kKmPHToMeterPS * diff/1000.0);
		}
		double gpsdist = 0.0;
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			long diff = next.time - cur.time;
			if(diff > 10000) {
				//Log.log("gap", cur, diff);
				continue;
			}
			double spd = cur.values[3];
			gpsdist += (spd * diff/1000.0);
		}
		Trace res = new Trace(3);
		res.values[0] = dist;
		res.values[1] = gpsdist;
		return res;
	}

}
