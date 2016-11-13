package gpsevaluation;

import io.DirectoryWalker;
import io.ReadWriteTrace;
import io.SqliteAccess;

import java.util.ArrayList;
import java.util.List;

import tracereplay.RealTimeBehaviorDetector;
import tracereplay.TraceReplayEngine;
import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class GPSDirection {
	

	private static String TAG = "GPSDirection";
		
	public static void start() {
		String outfolder = Constants.outputPath.concat("gpsdirection/turn/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath.concat("altima/urban"));
		List<Trace> gps = null;
		List<Trace> rotated_gyroscope = null;
				
		/*
		for(String directory: folders) {
			//Log.log(TAG, directory);
			List<Trip> trips = ReadWriteTrace.loadTrips(directory);
			for(Trip trip: trips) {
				Log.log(TAG, trip.path);
				gps = turnExtraction(trip.gps_elevation_);
				//rotated_gyroscope = initProjectGyroscope(trip);
				break;
			}
			break;
		}
		*/
		
		Trip trip = ReadWriteTrace.loadTrip(Constants.datPath.concat("lei/urban/1398642098259/"));
		Log.log(TAG, trip.path);
		
		gps = rawSteeringExtraction(trip.gps_elevation_);
		rotated_gyroscope = initProjectGyroscope(trip);
		
		ReadWriteTrace.writeFile(gps, outfolder.concat("gps.dat"));
		ReadWriteTrace.writeFile(rotated_gyroscope, outfolder.concat("rotated_gyroscope.dat"));
	}

	private static List<Trace> initProjectGyroscope(Trip trip) {
		List<Trace> accelerometer = trip.accelerometer_;
		List<Trace> gyroscope = trip.gyroscope_;
		List<Trace> rotation_matrix = trip.rotation_matrix_;
		
		List<Trace> rotated_gyroscope = new ArrayList<Trace>();
		
		
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		TraceReplayEngine.traceReplay(input, detector);
				
		Trace rm = detector.getInitRM();
		
		for(Trace trace: gyroscope) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace ntr = new Trace(3);
			ntr.copyTrace(initrot);
			rotated_gyroscope.add(ntr);
		}
		return rotated_gyroscope;
	}
	
	private static List<Trace> rawSteeringExtraction(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		Log.log(TAG, gps.get(0).toJson());
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			if(cur.values[3] == 0.0 && i > 0) {
				Trace pre = gps.get(i - 1);
				cur.values[2] = pre.values[2];
				continue;
			}
			double direction = GPSAbstraction.direction(cur, next);
			cur.values[2] = direction;
		}
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace ntr = new Trace(5);
			Trace cur = gps.get(i);
			Trace next = gps.get(i + 1);
			ntr.time = cur.time;
			for(int j = 0; j < cur.dim; ++j) {
				ntr.values[j] = cur.values[j];
			}
			ntr.values[4] = GPSAbstraction.turnAngle(cur.values[2], next.values[2])/((next.time - cur.time)/1000.0);
			res.add(ntr);
		}
		return res;
	}
	
	private static List<Trace> newSteeringExtraction(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> sampled = new ArrayList<Trace>();
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = null;
			int j = i + 1;
			double dist = 0.0;
			for(; j < gps.size(); ++j) {
				next = gps.get(j);
				double distbylatlng = GPSAbstraction.distance(cur, next);
				dist += distbylatlng;
				if(dist >= 10.0) {
					break;
				}
			}
			i = j - 1;
			double direction = GPSAbstraction.direction(cur, next);
			cur.values[2] = direction;
			sampled.add(cur);
		}
		for(int i = 0; i < sampled.size() - 1; ++i) {
			Trace ntr = new Trace(5);
			Trace cur = sampled.get(i);
			Trace next = sampled.get(i + 1);
			ntr.time = cur.time;
			for(int j = 0; j < cur.dim; ++j) {
				ntr.values[j] = cur.values[j];
			}
			ntr.values[4] = GPSAbstraction.turnAngle(cur.values[2], next.values[2])/((next.time - cur.time)/1000.0);
			res.add(ntr);
		}
		return res;
	}
}
