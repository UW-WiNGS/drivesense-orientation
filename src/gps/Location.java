package gps;

import java.util.List;

import utility.Constants;
import utility.Log;
import utility.Trace;

public class Location extends Trace {
	
	public double latitude = 0.0;
	public double longitude = 0.0;
	
	//public double distance = 0.0; //in meters
	public double speed = 0.0;//mph
	
	public int direction = 0;//0---360
	public int turn = 0; // -180 - 180, the difference to the next gps's direction
	
	public boolean stopped = false;

	public Location() {
		
	}
	
	public void calculateLocation(List<Trace> gps, int pos) {
		Trace cur = gps.get(pos);
		this.copyTrace(cur);
		latitude = cur.values[0];
		longitude = cur.values[1];
		double distance = 0.0;
		int npos = pos + 1;
		distance = GPSAbstraction.distance(cur, gps.get(npos));
		//speed is instant speed
		//if(distance < Constants.kGPSMinimumDistance)
		if(distance < 0.75 || gps.get(pos).values[2] == 0) //changed by bozhao	
			speed = 0.0;
		else
			speed = (distance*Constants.kMeterToMile)/((double)(gps.get(npos).time - cur.time)/(1000*60*60));
		//direction is not instant direction
		for(int i = pos + 1; i < gps.size(); ++i) {
			distance = GPSAbstraction.distance(cur, gps.get(i));
			if(distance > 10.0) {
				npos = i;
				break;
			}
		}
		
		direction = (int)GPSAbstraction.direction(cur, gps.get(npos));	
	}
	
	public void getLocation(Trace gps0, Trace gps1) {

		time = gps0.time;
		
		latitude = gps0.values[0];
		longitude = gps0.values[1];
		
		double duration = gps1.time - gps0.time;
		double distance = GPSAbstraction.distance(gps0, gps1);

		direction = (int)GPSAbstraction.direction(gps0, gps1);	
		
		if(distance < Constants.kGPSMinimumDistance) {
			speed = 0.0;
		} else {
			speed = (distance*Constants.kMeterToMile)/((double)duration/(1000*60*60));

		}
	}
	
	public void print() {
		Log.log("time:" + time + ", lat:" + latitude + ", long:" + longitude + ", direction:" + direction + 
				", speed: " + speed + ", turn: " + turn);
	}
	
}
