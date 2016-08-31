package gpselevation;

import java.util.List;

public class Elevation {
	class Location {
		double lat;
		double lng;
	}
	class Result {
		public double elevation;
		public Location location;
		public double resolution;
	}

	public List<Result> results;
	public String status;
}
