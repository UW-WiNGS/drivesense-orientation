package gpsevaluation;

import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PreProcess;
import utility.Trace;

public class GPSAbstraction {
	
	public static final String TAG = "GPSAbstraction";
	public static final double degreeError = 20;
	

	boolean isNaN (double x) {
		return x!=x;
	}
	/**
	 *@param gps0 (lat,long ...) 
	 *@param gps1 (lat, lomg..)
	 *@return the distance between two points, in meters
	 * based on haversine formula
	 */
	public static double distance(Trace gps0, Trace gps1) {
		
		double lat1 = Math.toRadians(gps0.values[0]);
		double lng1 = Math.toRadians(gps0.values[1]);
		double lat2 = Math.toRadians(gps1.values[0]);
		double lng2 = Math.toRadians(gps1.values[1]);
		/*
		double lat1 = Math.toRadians(gps0.values[1]);
		double lng1 = Math.toRadians(gps0.values[0]);
		double lat2 = Math.toRadians(gps1.values[1]);
		double lng2 = Math.toRadians(gps1.values[0]);
		*/
		double p1 = Math.cos(lat1)*Math.cos(lat2)*Math.cos(lng1-lng2);
		double p2 = Math.sin(lat1)*Math.sin(lat2);
		    
		double res = Math.acos(p1 + p2);
		if(res<Constants.kSmallEPSILON || res!=res) {
			res = 0.0;
		}

		//Log.log("dis:", res);
		return res * Constants.kEarthRadius;
	}
	
	public static Trace directionVector(Trace gps0, Trace gps1) {
		double lat1 = Math.toRadians(gps0.values[0]);
		double lon1 = Math.toRadians(gps0.values[1]);
		double lat2 = Math.toRadians(gps1.values[0]);
		double lon2 = Math.toRadians(gps1.values[1]);

		double y = Math.cos(lat1)*Math.sin(lat2)-Math.sin(lat1)*Math.cos(lat2)*Math.cos(lon2-lon1);
		double x = Math.sin(lon2 - lon1)*Math.cos(lat2);
		
		Trace res = new Trace(3);
		double sum = Math.sqrt(Math.pow(x, 2.0) + Math.pow(y, 2.0));
		res.setValues(x/sum, y/sum, 0.0);
		return res;
	}
	
	/**
	 * 0 for East
	 * 90 for North
	 * 180 for West
	 * 270 for South
	 * */
	public static double direction(Trace gps0, Trace gps1) {
		double lat1 = Math.toRadians(gps0.values[0]);
		double lon1 = Math.toRadians(gps0.values[1]);
		double lat2 = Math.toRadians(gps1.values[0]);
		double lon2 = Math.toRadians(gps1.values[1]);

		double y = Math.cos(lat1)*Math.sin(lat2)-Math.sin(lat1)*Math.cos(lat2)*Math.cos(lon2-lon1);
		double x = Math.sin(lon2 - lon1)*Math.cos(lat2);
		double res = Math.atan2(y, x);
		//atan2(cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1), sin(lon2-lon1)*cos(lat2))
		double degree = Math.toDegrees(res);
		
		if(degree < 0.0) degree+=360.0;
		if(degree >= 360.0 || degree < 0.0) {
			Log.error("direction error:", degree);
		}
		return degree;
	}
	
	
	/**
	 * right turn angle is negative, left turn angle is positive
	 * @param degree1
	 * @param degree2
	 * @return
	 */
	public static double turnAngle(double degree1, double degree2) {
		assert degree1 >= 0 && degree1 < 360 && degree2 >= 0 && degree2 < 360;
		
		//vector way
		double x1 = Math.cos(Math.toRadians(degree1));
		double y1 = Math.sin(Math.toRadians(degree1));
		double x2 = Math.cos(Math.toRadians(degree2));
		double y2 = Math.sin(Math.toRadians(degree2));
		
		double angle = Math.atan2(y2, x2) - Math.atan2(y1, x1);
		//Log.log(TAG, angle, Math.toDegrees(angle));
		
		//simple math way
		double res = degree2 - degree1;
		if(res > 180) {
			res -= 360;
		} else if(res < -180) {
			res += 360;
		} else {
			
		}
		return res;
	}
	/**
	 * 
	 * @param gps
	 * @return
	 */
	public static double accumulatedDistance(List<Trace> gps) {
		double dist = 0.0;
		int sz = gps.size();
		for(int i = 0; i < sz - 1; ++i) {
			dist += distance(gps.get(i), gps.get(i + 1));
		}
		//in miles
		return dist * Constants.kMeterToMile;
	}
	

	/**
	 * 
	 * @param gps
	 * @return m/s
	 */
	public static List<Trace> speedmap(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		int sz = gps.size();
		for(int i = 0; i < sz - 1; ++i) {
			Trace trace = new Trace();
			trace.copyTrace(gps.get(i));
			double dis = distance(gps.get(i), gps.get(i+1));
			//Log.log(dis);
			double t = (gps.get(i+1).time - gps.get(i).time)/1000.0;
			trace.values[2] = dis/t;
			res.add(trace);
		}
		return res;
	}
	
	/**
	 * 
	 * @param gps (with speed on the 3rd value)
	 * @return
	 */
	public static List<Trace> removeJitter(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		int sz = gps.size();
		for(int i = 1; i < sz - 1; ++i) {
			Trace trace = new Trace();
			trace.copyTrace(gps.get(i));
			if(Math.abs(trace.values[2] - gps.get(i - 1).values[2]) > 5.0 
					&& Math.abs(trace.values[2] - gps.get(i + 1).values[2]) > 2.0) {
				trace.values[2] = (gps.get(i - 1).values[2] + gps.get(i + 1).values[2])/2.0;
			}
			
			res.add(trace);
		}
		return res;
	}
	/**
	 * 
	 * 
	 * @param gps, the third value should be speed, m/s
	 * @return m/s^2
	 */
	public static List<Trace> calculateAcceleration(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		int sz = gps.size();
		for(int i = 0; i < sz - 1; ++i) {
			Trace trace = new Trace();
			trace.copyTrace(gps.get(i));
			double speed_diff = gps.get(i + 1).values[2] - gps.get(i).values[2];
			double t = (gps.get(i+1).time - gps.get(i).time)/1000.0;
			trace.values[2] = speed_diff/t;
			res.add(trace);
		}
		return res;
	}	
	
	/**
	 * 
	 * @param gps
	 * @return
	 */
	public static List<Trace> calculateAngluarChanges(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		int sz = gps.size();
		for(int i = 0; i < sz - 1; ++i) {
			Trace trace = new Trace();
			trace.copyTrace(gps.get(i));
			double angle = direction(gps.get(i), gps.get(i + 1));
			trace.values[2] = Math.toRadians(angle);
			res.add(trace);
		}
		return res;
	}

	public static List<Trace> wrapperGPS(List<Trace> gps) {
		List<Trace> res = new ArrayList<Trace>();
		for(int i = 0; i < gps.size() - 1; ++i) {
			Trace cur = gps.get(i);
			Trace ntr = new Trace(6);
			ntr.time = cur.time;
			ntr.type = cur.type;
			for(int j = 0; j < cur.dim; ++j) {
				ntr.values[j] = cur.values[j];
			}			
			ntr.values[3] = (gps.get(i + 1).values[2] - cur.values[2])/((gps.get(i + 1).time - cur.time)/1000.0);
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
	
	/**
	 * assume that the difference between OBD speed and gps is within 10 seconds
	 * @param speed
	 * @param gps: 
	 * @param initshift: the initial time difference between gps and obd speed
	 * @return -1 on failure
	 */
	public static long sychronization(List<Trace> speed, List<Trace> gps) {
		if(gps.size() < 10) {
			return -1;
		}
		Trace first = gps.get(0);
		Trace last = gps.get(gps.size() - 1);
		if(GPSAbstraction.distance(first, last) < 100) {
			return -1;
		}
		double rate = 2.0;
		List<Trace> spds = PreProcess.interpolate(speed, rate);
		final long diff = -100000;
		long desireshift = 0;
		double minerr = Double.MAX_VALUE;
		for(long shift = diff; shift <= Math.abs(diff); shift += 250) {
			double sumabserr = 0.0;
			double sumerr = 0.0;
			for(int i = 0; i < gps.size() - 1; ++i) {
				Trace cur = gps.get(i);
				Trace next = gps.get(i + 1);
				long time = cur.time + shift;
				double speedinmps = GPSAbstraction.distance(cur, next)/((next.time - cur.time)/1000.0);
				Trace spd = PreProcess.getTraceAt(spds, time);
				double err = speedinmps;
				if(spd!=null) {
					err = spd.values[0] * Constants.kKmPHToMeterPS - speedinmps;
				}
				sumerr += err;
				sumabserr += Math.abs(err);
				
			}
			if(sumabserr < minerr) {
				minerr = sumabserr;
				desireshift = shift;
			}
		}
		return desireshift;
	}
}
