package gpsevaluation;

import io.DirectoryWalker;
import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;

import tracereplay.RealTimeBehaviorDetector;
import tracereplay.TraceReplayEngine;
import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.Pattern;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class GPSDirection {
	

	private static String TAG = "GPSDirection";
	private static boolean DEBUG = true;
		
	public static void start() {
		String outfolder = Constants.outputPath.concat("gpsdirection/comparelanechange/");
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
		
		Trip trip = ReadWriteTrace.loadTrip(Constants.datPath.concat("lei/urban/1398645857108/"));
		
		
		Log.log(TAG, trip.path);
		List<Trace> raw = steeringExtractionByGPS(trip.gps_elevation_);
		
		//gps = steeringExtractionByGPS(trip.gps_elevation_);
		
		gps = PreProcess.exponentialMovingAverage(raw, 5);
		
		rotated_gyroscope = initProjectGyroscope(trip);
		
		/*
		List<Pattern> turns = TurnExtraction.extractTurns(rotated_gyroscope);		
		compareWithGPSForTurns(gps, turns);
		*/
		List<Pattern> lanechanges = LanechangeExtraction.extractLanechanges(rotated_gyroscope);
		compareWithGPSForLanechange(gps, lanechanges);
		
		
		ReadWriteTrace.writeFile(gps, outfolder.concat("gps.dat"));
		ReadWriteTrace.writeFile(rotated_gyroscope, outfolder.concat("rotated_gyroscope.dat"));
		
		
		//goOverAllTheDataForTurns();
		//Log.log(TAG, (double)found/total);
	}
	
	
	private static void goOverAllTheDataForTurns() {
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		List<Trace> output = new ArrayList<Trace>();
		String type = "highway";
		for(String directory: folders) {
			Log.log(TAG, directory);
			String folder = directory.concat("/" + type);
			List<Trip> trips = ReadWriteTrace.loadTrips(folder);
			for(Trip trip: trips) {
				List<Trace> gps = PreProcess.exponentialMovingAverage(steeringExtractionByGPS(trip.gps_elevation_), 5);
				List<Trace> rotated_gyroscope = initProjectGyroscope(trip);
				List<Pattern> turns = TurnExtraction.extractTurns(rotated_gyroscope);
				
				Log.log(TAG, trip.path);
				List<Trace> tmp = compareWithGPSForTurns(gps, turns);
				output.addAll(tmp);
			}
		}
		String outfolder = Constants.outputPath.concat("gpsdirection/compareturn/");
		ReadWriteTrace.writeFile(output, outfolder.concat(type + "_turns.dat"));
	}
	
	private static List<Trace> compareWithGPSForLanechange(List<Trace> processedgps, List<Pattern> lanechanges) {
		List<Trace> res = new ArrayList<Trace>();
		long gpsstart = -1, gpsend = -1;
		for(int i = 0; i < processedgps.size(); ++i) {
			Trace cur = processedgps.get(i);
			if(cur.values[3] != 0.0) {
				gpsstart = cur.time;
				break;
			}
		}
		for(int i = processedgps.size() - 1; i >= 0; --i) {
			Trace cur = processedgps.get(i);
			if(cur.values[3] != 0.0) {
				gpsend = cur.time;
				break;
			}
		}
		for(Pattern lanechange: lanechanges) {
			long start = lanechange.start - 5000;
			long end = lanechange.end + 5000;
			if(start <= gpsstart + 10 * 1000 || end >= gpsend - 10 * 1000 || end - start < 10*1000) continue;
			List<Trace> subgps = PreProcess.extractSubList(processedgps, start, end);
			int sz = subgps.size();
			if(sz <= 5) {
				continue;
			}
			
			
			Log.log(TAG, start, end);
			/*
			Trace ntr = new Trace(2);
			ntr.values[0] = turn.accumulated_change;
			ntr.values[1] = gpsaccum;
			res.add(ntr);
			*/
		}
		return res;
		
	}
	
	
	private static int total = 0;
	private static int found = 0;
	private static List<Trace> compareWithGPSForTurns(List<Trace> processedgps, List<Pattern> turns) {
		List<Trace> res = new ArrayList<Trace>();
		long gpsstart = -1, gpsend = -1;
		for(int i = 0; i < processedgps.size(); ++i) {
			Trace cur = processedgps.get(i);
			if(cur.values[3] != 0.0) {
				gpsstart = cur.time;
				break;
			}
		}
		for(int i = processedgps.size() - 1; i >= 0; --i) {
			Trace cur = processedgps.get(i);
			if(cur.values[3] != 0.0) {
				gpsend = cur.time;
				break;
			}
		}
		for(Pattern turn: turns) {
			long start = turn.start - 5000;
			long end = turn.end + 5000;
			if(start <= gpsstart + 10 * 1000 || end >= gpsend - 10 * 1000 || end - start < 10*1000) continue;
			List<Trace> subgps = PreProcess.extractSubList(processedgps, start, end);
			int sz = subgps.size();
			if(sz <= 10) {
				continue;
			}
			double sdir = 0.0, edir = 0.0;
			for(int i = 0; i < 3; ++i) {
				sdir += subgps.get(i).values[4];
			}
			for(int i = 0; i < 3; ++i) {
				edir += subgps.get(sz - 1 - i).values[4];
			}
			sdir/=3.0;
			edir/=3.0;
			double gpsaccum = GPSAbstraction.turnAngle(sdir, edir);
			
			if(Math.abs(turn.accumulated_change) >= 120) continue;
			
			if(Math.abs(turn.accumulated_change + gpsaccum) < Math.abs(turn.accumulated_change - gpsaccum)) {
				gpsaccum *= -1;
			}
			
			if(Math.abs(turn.accumulated_change - gpsaccum) >= 60) {
				continue;
			}
			Log.log(TAG, turn.start, turn.end);
			Log.log(TAG, turn.accumulated_change, gpsaccum);

			total++;
			if(Math.abs(gpsaccum) >= 60) {
				found++;
			}
			Trace ntr = new Trace(2);
			ntr.values[0] = turn.accumulated_change;
			ntr.values[1] = gpsaccum;
			res.add(ntr);
		}
		return res;
	}

	private static List<Trace> initProjectGyroscope(Trip trip) {
		List<Trace> accelerometer = trip.accelerometer_;
		List<Trace> gyroscope = PreProcess.exponentialMovingAverage(trip.gyroscope_, -1);
		List<Trace> rotation_matrix = trip.rotation_matrix_;
		
		List<Trace> rotated_gyroscope = new ArrayList<Trace>();
		
		
		List<List<Trace>> input = new ArrayList<List<Trace>>();
		input.add(accelerometer);
		input.add(gyroscope);
		input.add(rotation_matrix);
		
		RealTimeBehaviorDetector detector = new RealTimeBehaviorDetector();
		TraceReplayEngine.traceReplay(input, detector);
				
		Trace rm = detector.getInitRM();
		if(rm == null) {
			return rotated_gyroscope;
		}
		
		for(Trace trace: gyroscope) {
			Trace initrot = Formulas.rotate(trace, rm.values);
			Trace ntr = new Trace(3);
			ntr.copyTrace(initrot);
			rotated_gyroscope.add(ntr);
		}
		return rotated_gyroscope;
	}
	
	/**
	 * lat, lng, alt, speed, direction, angular speed
	 * @param gps
	 * @return
	 */
	
	private static List<Trace> steeringExtractionByGPS(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace ntr = new Trace(6);
			ntr.time = cur.time;
			ntr.type = cur.type;
			for(int j = 0; j < cur.dim; ++j) {
				ntr.values[j] = cur.values[j];
			}			
			for(int j = i + 1; j < gps.size(); ++j) {
				Trace next = gps.get(j);
				double dist = GPSAbstraction.distance(cur, next);
				if(dist <= 10.0) continue;
				double direction = GPSAbstraction.direction(cur, next);
				ntr.values[4] = direction;
				break;			
			}
			int sz = res.size();
			if(sz >= 1) {
				Trace pre = res.get(sz - 1);
				pre.values[5] = GPSAbstraction.turnAngle(pre.values[4], ntr.values[4])/((ntr.time - pre.time)/1000.0);
				
				if(pre.time >= 110171 - 5000 && pre.time <= 116848 + 5000 && DEBUG == true)
					Log.log(TAG, pre);
			}
			res.add(ntr);
		}
		return res;
	}
	
	
}
