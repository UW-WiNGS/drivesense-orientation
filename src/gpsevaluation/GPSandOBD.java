package gpsevaluation;

import io.DirectoryWalker;
import io.ReadWriteTrace;

import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;
import utility.Trip;

public class GPSandOBD {
	
	private static String TAG = "GPSEvaluation";
	
	public static void start() {
		String outfolder = Constants.outputPath.concat("gpsevaluation/compareobdgps/accelerationofspeeds/");
		List<String> folders = DirectoryWalker.getFolders(Constants.datPath);
		double sum = 0.0;
		List<Trace> output = new ArrayList<Trace>();
		List<List<Trace>> dict = new ArrayList<List<Trace>>();
		for(int i = 0; i < 4; i++) {
			List<Trace> cur = new ArrayList<Trace>();
			dict.add(cur);
		}
		String type = "highway";
		for(String directory: folders) {
			//Log.log(TAG, directory);
			String folder = directory.concat("/" + type);
			List<Trip> trips = ReadWriteTrace.loadTrips(folder);
			for(Trip trip: trips) {
				//Trace cur = checkMissingGPS(trip);
				//output.add(cur);
				//List<Trace> cur = compareGPSAndOBDSpeed(trip);
				//List<Trace> cur = compareGPSAndOBDAcceleration(trip);
				//List<Trace> cur = compareHardAcceleration(trip, 2.0);
				//output.addAll(cur);
				compareAccelerationOfSpeeds(trip, dict);
			}
			//break;
		}
		//Log.log(sum);
		//ReadWriteTrace.writeFile(output, outfolder.concat(type + ".dat"));
		//double corr = Formulas.linear_correlation(output, 2, 3);
		//Log.log(corr);
		
		for(int i = 0; i < 4; ++i) {
			ReadWriteTrace.writeFile(dict.get(i), outfolder.concat(type + i + ".dat"));	
		}
		
	}
	

	private static void compareAccelerationOfSpeeds(Trip trip, List<List<Trace>> dict) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> speed = PreProcess.interpolate(trip.speed_, 2.0);
		List<Trace> gps = trip.gps_elevation_;
		for(int i = 0 ; i < gps.size() - 1; ++i) {
			Trace curgps = gps.get(i);
			Trace nextgps = gps.get(i + 1);
			double gpsacce = (nextgps.values[3] - curgps.values[3])/((nextgps.time - curgps.time)/1000.0);
			
			Trace curobd = PreProcess.getTraceAt(speed, curgps.time);
			Trace nextobd = PreProcess.getTraceAt(speed, nextgps.time);
			if(curobd == null || nextobd == null || nextobd.time == curobd.time) continue;
			double obdacce = (nextobd.values[0] - curobd.values[0]) * Constants.kKmPHToMeterPS/((nextobd.time - curobd.time)/1000.0);
			
			Trace cur = new Trace(4);
			cur.time = curgps.time;
			cur.values[0] = gpsacce;
			cur.values[1] = obdacce;
			cur.values[2] = gpsacce - obdacce;
			double curspeed = curobd.values[0] * Constants.kKmPHToMeterPS;
			cur.values[3] = curspeed;
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

	
	private static List<Trace> compareHardAcceleration(Trip trip, double threshold) {
		List<Trace> res = new ArrayList<Trace>();
		List<Trace> speed = trip.speed_;
		List<Trace> gps = trip.gps_elevation_;
		if(gps.size() <= 10) {
			return res;
		}

		for(int i = 0; i < speed.size() - 1; ++i) {
			Trace curobd = speed.get(i);
			Trace nextobd = speed.get(i + 1);
			double obdacce = (nextobd.values[0] - curobd.values[0]) * Constants.kKmPHToMeterPS/((nextobd.time - curobd.time)/1000.0);
			if(Math.abs(obdacce) >= threshold) {
				long start = curobd.time - 5*1000;
				List<Trace> subgps = PreProcess.extractSubList(gps, start, start + 10 * 1000);
				if(subgps.size() < 8) continue;
				double mindiff = 10.0;
				Trace ntr = new Trace(4);
				ntr.time = curobd.time;
				ntr.values[0] = obdacce;
				ntr.values[3] = curobd.values[0] * Constants.kKmPHToMeterPS;
				for(int j = 0; j < subgps.size() - 1; ++j) {
					Trace curgps = subgps.get(j);
					Trace nextgps = subgps.get(j + 1);
					double gpsacce = (nextgps.values[3] - curgps.values[3])/((nextgps.time - curgps.time)/1000.0);
					if(Math.abs(obdacce - gpsacce) < mindiff) {
						mindiff = Math.abs(obdacce - gpsacce);						
						ntr.values[1] = gpsacce;
						ntr.values[2] = obdacce - gpsacce;
					}
				}
				res.add(ntr);
				
			}
		}
		return res;
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
			if(curobd == null || nextobd == null || nextobd.time == curobd.time) continue;
			double obdacce = (nextobd.values[0] - curobd.values[0]) * Constants.kKmPHToMeterPS/((nextobd.time - curobd.time)/1000.0);
			
			Trace cur = new Trace(3);
			cur.time = curgps.time;
			cur.values[0] = gpsacce;
			cur.values[1] = obdacce;
			cur.values[2] = gpsacce - obdacce;

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
