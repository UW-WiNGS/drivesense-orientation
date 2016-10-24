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
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		String type = "urban";
		
		List<Trace> gps = null;
		List<Trace> rotated_gyroscope = null;
		for(String directory: folders) {
			//Log.log(TAG, directory);
			String folder = directory.concat("/" + type);
			List<Trip> trips = ReadWriteTrace.loadTrips(folder);
			for(Trip trip: trips) {
				Log.log(TAG, trip.path);
				gps = turnExtraction(trip.gps_elevation_);
				//rotated_gyroscope = initProjectGyroscope(trip);
				break;
			}
			break;
		}
		
		ReadWriteTrace.writeFile(gps, outfolder.concat("gps.dat"));
		//ReadWriteTrace.writeFile(rotated_gyroscope, outfolder.concat("rotated_gyroscope.dat"));
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
	
	private static List<Trace> turnExtraction(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace next = null;
			int j = i + 1;
			double dist = 0.0;
			for(; j < gps.size(); ++j) {
				next = gps.get(j);
				dist += Math.min(GPSAbstraction.distance(cur, next), cur.values[3] * (next.time - cur.time)/1000.0);
				if(dist >= 50.0) {
					break;
				}
			}
			double direction = GPSAbstraction.direction(cur, next);
			cur.values[2] = direction;
		}
		return gps;
	}
}
