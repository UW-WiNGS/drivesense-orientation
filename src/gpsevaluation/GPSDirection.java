package gpsevaluation;

import io.DirectoryWalker;
import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;

import sensors.OrientationChangeDetection;
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
		compareGPSandGyroSteeringMotions();
		
	}
	private static void GPSSteeringExample() {
		String outfolder = Constants.outputPath.concat("gpsdirection/comparelanechange/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath.concat("altima/urban"));
		List<Trace> gps = null;
		List<Trace> rotated_gyroscope = null;
		
		Trip trip = ReadWriteTrace.loadTrip(Constants.datPath.concat("lei/highway/1423344301931/"));
		
		Log.log(TAG, trip.path);
		List<Trace> raw = steeringExtractionByGPS(trip.gps_elevation_);
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
		
	}
	
	
	public static void compareGPSandGyroSteeringMotions() {
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		String outpath = Constants.kAlterSenseOutput.concat("gpsdirection/steering/");
		
		List<List<Trace>> dict = new ArrayList<List<Trace>>();
		for(int i = 0; i < 4; i++) {
			List<Trace> cur = new ArrayList<Trace>();
			dict.add(cur);
		}
		
		for(String directory: folders) {
			List<Trace> output = new ArrayList<Trace>();
			String names[] = directory.split("/");
			//Log.log(TAG, directory);
			String highway = directory.concat("/highway");
			String urban = directory.concat("/urban");
			
			List<Trip> htrips = ReadWriteTrace.loadTrips(highway);
			
			List<Trip> trips = ReadWriteTrace.loadTrips(urban);
			trips.addAll(htrips);
			for(Trip trip: trips) {
				List<Trace> accelerometer = PreProcess.exponentialMovingAverage(trip.accelerometer_, -1);
				if(accelerometer.size() < 600 || OrientationChangeDetection.orientationChanged(accelerometer)) {
					//Log.log(TAG, trip.path, "too short");
					continue;
				}
				
				List<Trace> raw = steeringExtractionByGPS(trip.gps_elevation_);
				List<Trace> gps = PreProcess.exponentialMovingAverage(raw, 5);
				
				List<Trace> rotated_gyroscope = initProjectGyroscope(trip);
				
				compareSteeringOfSpeeds(gps, rotated_gyroscope, dict);

			}
		}
		for(int i = 0; i < 4; ++i) {
			ReadWriteTrace.writeFile(dict.get(i), outpath.concat(i + ".dat"));	
		}
	}
	
	private static void compareSteeringOfSpeeds(List<Trace> gps, List<Trace> gyro, List<List<Trace>> dict) {
		List<Trace> res = new ArrayList<Trace>();
		int counter = 0;
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace curgps = gps.get(i);
			double steer = curgps.values[5];
			Trace curgyro = PreProcess.getTraceAt(gyro, curgps.time);
			if(curgyro == null) continue;
			Trace cur = new Trace(1);
			cur.values[0] = curgps.values[5]/Constants.kRadianToDegree - curgyro.values[2];
			
			if(Math.abs(cur.values[0]) > 0.4) {
				counter++;
			}
		}
		if(counter >= gps.size()/10) {
			return;
		}
		
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace curgps = gps.get(i);
			double steer = curgps.values[5];
			Trace curgyro = PreProcess.getTraceAt(gyro, curgps.time);
			if(curgyro == null) continue;
			Trace cur = new Trace(3);
			cur.values[0] = curgps.values[5]/Constants.kRadianToDegree - curgyro.values[2];
			cur.values[1] = curgps.values[5]/Constants.kRadianToDegree;
			cur.values[2] = curgyro.values[2];
			
			if(Math.abs(cur.values[2]) < 0.02) {
				continue;
			}
	
			double curspeed = curgps.values[3];			
			int j = -1;
			if(curspeed == 0.0) {
				j = 0;
			} else if(curspeed < 10.0) {
				j = 1;
			} else if(curspeed < 20.0) {
				j = 2;
			} else {
				j = 3;
			}
			dict.get(j).add(cur);
		}
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
	
	public static List<Trace> steeringExtractionByGPS(List<Trace> gps) {
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
			}
			res.add(ntr);
		}
		return res;
	}
	
	
}
