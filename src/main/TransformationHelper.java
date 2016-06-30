package main;

import java.util.ArrayList;
import java.util.List;

import utility.Constants;
import utility.Formulas;
import utility.Log;
import utility.PairDouble;
import utility.Trace;

public class TransformationHelper {
	
	private Trace rotation_matrix = new Trace(9);
	private PairDouble newAxes[] = new PairDouble[2];
	private boolean reversed = false;
	
	
	private boolean meta_set = false;
	public TransformationHelper() {}
	
	public TransformationHelper(Trace rm, PairDouble na[], boolean rev) {
		assert rm.dim == 9;
		rotation_matrix.copyTrace(rm);
		assert na.length == 2;
		newAxes = na.clone();
		reversed = rev;
		
		meta_set = true;
	}
	public static PairDouble[] AxesUnitVector(ArrayList<PairDouble> raw_xys, double slope) {
		PairDouble unit_x = new PairDouble();
		PairDouble unit_y = new PairDouble();

		int rightnum = 0;
		double perpendicular_slope = -1/slope;

		for (int i = 0; i < raw_xys.size(); i++) {
			PairDouble xy = raw_xys.get(i);
			if (slope > 0 ^ xy.x * perpendicular_slope > xy.y)
				rightnum ++;
		}
		

		int y_indicator = (rightnum/raw_xys.size() > Constants.PERCENT_)? 1:-1;
		int x_indicator = ((y_indicator > 0) ^ (slope > 0))? -1:1;

		unit_x = Formulas.UnitVector( new PairDouble(x_indicator, x_indicator*perpendicular_slope) );
		unit_y = Formulas.UnitVector( new PairDouble(y_indicator, y_indicator*slope) );

		PairDouble[] res = {unit_x, unit_y};

		return res;
	}
	public List<Trace> transformByAccelerometer(List<Trace> accelerometer, Trace rotation) {
		List<Trace> calculated = CoordinateTransformation.Calculate(accelerometer, rotation, "rotation");
		rotation_matrix.copyTrace(rotation);
		
		//List<Trace> head_straight = CoordinateTransformation.getHeadingStraightTraces(calculated);
		ArrayList<PairDouble> raw_xy = new ArrayList<PairDouble>();

		for (Trace t : calculated) {
			PairDouble xy = new PairDouble(t.values[0],  t.values[1]);
			raw_xy.add(xy);
		}
		double slope = BestFitOrigin(raw_xy);
		/*two unit vectors of mapped x and y*/
		PairDouble[] rawAxes = AxesUnitVector(raw_xy, slope);
		newAxes = rawAxes.clone();
		//adjust the coordinate system
		ArrayList<Trace> mapped = new ArrayList<Trace>();
		for (Trace tr : calculated) {
			PairDouble vec = new PairDouble(tr.values[0], tr.values[1]);
			Trace newtr = new Trace();
			newtr.time = tr.time;
			newtr.values[0] = Formulas.DotProduct(vec, rawAxes[0]);
			newtr.values[1] = Formulas.DotProduct(vec, rawAxes[1]);
			newtr.values[2] = tr.values[2];
			mapped.add(newtr);
		}
		/*
		if(CoordinateTransformation.isCoordinateMappingReversed(mapped))
		{
			reversed = true;
			for(Trace t : mapped)
			{
				t.values[1] *= -1;
				t.values[0] *= -1;
			}
		}
		*/
		meta_set = true;
		return mapped;
	}
	public static double BestFitOrigin(ArrayList<PairDouble> points) {
		int n = points.size();
		double sum_xy, sum_x2, sum_y2;
		sum_xy = sum_x2 = sum_y2 = 0;
		for (int i = 0; i < n; i++) {
			double x = points.get(i).x;
			double y = points.get(i).y;
			sum_xy += x*y;
			sum_x2 += Math.pow(x, 2);
			sum_y2 += Math.pow(y, 2);
		}

		double temp = sum_y2 - sum_x2;

		// get both + and - slopes.
		double slope1 = (temp + Math.sqrt(Math.pow(temp, 2) + 4 * Math.pow(sum_xy, 2) )) / (2 * sum_xy);
		double slope2 = (temp - Math.sqrt(Math.pow(temp, 2) + 4 * Math.pow(sum_xy, 2) )) / (2 * sum_xy);
		// plug into the distance calculation equation to compare which one is smaller.
		double ds1 = Formulas.DistanceSquare(slope1, sum_x2, sum_y2, sum_xy);
		double ds2 = Formulas.DistanceSquare(slope2, sum_x2, sum_y2, sum_xy);
		return (ds1 < ds2)? slope1 : slope2;
	}
	
	public List<Trace> transform (List<Trace> raw) {
		List<Trace> projected = new ArrayList<Trace>();
		if(raw.size()==0) {
			return projected;
		}
		if(meta_set==false) {
			Log.log("TransformationHelper has not been set yet!");
			return projected;
		}
		double factor = 9.4;
		List<Trace> input = CoordinateTransformation.Calculate(raw, rotation_matrix, "rotation");
		
		int coeff = 1;
		if(reversed) coeff = -1;
		for (Trace tr : input) {
			PairDouble vec = new PairDouble(tr.values[0], tr.values[1]);
			Trace newtr = new Trace();
			newtr.time = tr.time;
			newtr.values[0] = coeff * Formulas.DotProduct(vec, newAxes[0]);
			newtr.values[1] = coeff * Formulas.DotProduct(vec, newAxes[1]);
			newtr.values[2] = tr.values[2];
			projected.add(newtr);
		}
		return projected;
	}
}
