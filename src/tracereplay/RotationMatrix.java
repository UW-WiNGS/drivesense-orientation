package tracereplay;

import java.util.List;

import utility.Trace;

public class RotationMatrix {

	    private long tripid_;


	    private long time_;

	    private double x0y0_;

	    private double x0y1_;

	    private double x0y2_;

	    private double x1y0_;
	
	    private double x1y1_;
	
	    private double x1y2_;

	    private double x2y0_;

	    private double x2y1_;

	    private double x2y2_;
	    

	    private double projx0_;

	    private double projy0_;

	    private double projx1_;

	    private double projy1_;
	    
		public double [] rotation_matrix = {};
		public PairDouble newAxes[] = new PairDouble[2];
		public boolean reversed = false;
		
		public boolean rm_set = false;
		public boolean aligned = false;
		public boolean all_set = false;
	    

	    public void setTripID(long id) {
	        this.tripid_ = id;
	    }

	    public RotationMatrix() {

	    }

	    public void setRotationMatrix(Trace reading) {
	        this.time_ = System.currentTimeMillis();
	        this.x0y0_ = reading.values[0];
	        this.x0y1_ = reading.values[1];
	        this.x0y2_ = reading.values[2];
	        this.x1y0_ = reading.values[3];
	        this.x1y1_ = reading.values[4];
	        this.x1y2_ = reading.values[5];
	        this.x2y0_ = reading.values[6];
	        this.x2y1_ = reading.values[7];
	        this.x2y2_ = reading.values[8];
	    }

	    public void setRotationMatrix(float [] rm) {
	        this.time_ = System.currentTimeMillis();
	        this.x0y0_ = rm[0];
	        this.x0y1_ = rm[1];
	        this.x0y2_ = rm[2];
	        this.x1y0_ = rm[3];
	        this.x1y1_ = rm[4];
	        this.x1y2_ = rm[5];
	        this.x2y0_ = rm[6];
	        this.x2y1_ = rm[7];
	        this.x2y2_ = rm[8];
	    }

	    /**
		 * Using rotational matrix.
		 * @param raw_tr
		 * @param rM
		 * @return
		 */
		public Trace Rotate(Trace raw_tr) {
			Trace calculated_tr = new Trace();
			calculated_tr.time = raw_tr.time;
			double x, y, z;
			x = raw_tr.values[0];
			y = raw_tr.values[1];
			z = raw_tr.values[2];

			calculated_tr.values[0] = x * x0y0_ + y * x0y1_ + z * x0y2_;
			calculated_tr.values[1] = x * x1y0_ + y * x1y1_ + z * x1y2_;
			calculated_tr.values[2] = x * x2y0_ + y * x2y1_ + z * x2y2_;

			return calculated_tr;
		}

		public Trace Alignment(Trace raw) {
			Trace ntr = new Trace();
			PairDouble vec = new PairDouble(raw.values[0], raw.values[1]);
			ntr.time = raw.time;
			ntr.values[0] = -1.0 * vec.DotProduct(new PairDouble(projx0_, projy0_));
			ntr.values[1] = -1.0 * vec.DotProduct(new PairDouble(projx1_, projy1_));
			ntr.values[2] = raw.values[2];			
			return ntr;
		}
		
		 /**
	     *
	     * @param points
	     * @return
	     */
	    public static double curveFit(List<Trace> points) {
	        int n = points.size();
	        double sum_xy, sum_x2, sum_y2;
	        sum_xy = sum_x2 = sum_y2 = 0;
	        for (int i = 0; i < n; i++) {
	            double x = points.get(i).values[0];
	            double y = points.get(i).values[1];
	            sum_xy += x*y;
	            sum_x2 += Math.pow(x, 2);
	            sum_y2 += Math.pow(y, 2);
	        }

	        double temp = sum_y2 - sum_x2;

	        // get both + and - slopes.
	        double slope1 = (temp + Math.sqrt(Math.pow(temp, 2) + 4 * Math.pow(sum_xy, 2) )) / (2 * sum_xy);
	        double slope2 = (temp - Math.sqrt(Math.pow(temp, 2) + 4 * Math.pow(sum_xy, 2) )) / (2 * sum_xy);
	        // plug into the distance calculation equation to compare which one is smaller.
	        double ds1 = DistanceSquare(slope1, sum_x2, sum_y2, sum_xy);
	        double ds2 = DistanceSquare(slope2, sum_x2, sum_y2, sum_xy);
	        return (ds1 < ds2)? slope1 : slope2;
	    }

	    private static double DistanceSquare(double slope, double sum_x2, double sum_y2, double sum_xy) {
	        double res = (sum_y2 - 2 * slope * sum_xy + Math.pow(slope, 2) * sum_x2) / (Math.pow(slope, 2) + 1);
	        return res;
	    }

	    public void setUnitVector(List<Trace> raw_xys, double slope) {
	        if(slope==0.0) slope = 0.01;
	        PairDouble unit_x = new PairDouble();
	        PairDouble unit_y = new PairDouble();

	        int rightnum = 0;
	        double perpendicular_slope = -1/slope;

	        for (int i = 0; i < raw_xys.size(); i++) {
	            Trace xy = raw_xys.get(i);
	            if (slope > 0 ^ xy.values[0] * perpendicular_slope > xy.values[1])
	                rightnum ++;
	        }


	        int y_indicator = (rightnum/raw_xys.size() > 0.7)? 1:-1;
	        int x_indicator = ((y_indicator > 0) ^ (slope > 0))? -1:1;

	        unit_x = UnitVector( new PairDouble(x_indicator, x_indicator*perpendicular_slope) );
	        unit_y = UnitVector( new PairDouble(y_indicator, y_indicator*slope) );

	        PairDouble[] res = {unit_x, unit_y};

	        projx0_ = x_indicator;
	        projy0_ = x_indicator*perpendicular_slope;


	        projx1_ = y_indicator;
		    projy1_ = y_indicator*slope;
	        
	        
	        //return res;
	    }

	    public class PairDouble {
	        public double x;
	        public double y;
	        public PairDouble() {
	            x = 0;
	            y = 0;
	        }
	        public PairDouble(double x, double y) {
	            this.x = x;
	            this.y = y;
	        }
	        
	    	public double DotProduct(PairDouble u) {
	    		return x * u.x + y * u.y;
	    	}
	    }
	    public PairDouble UnitVector(PairDouble v) {
	        double length = Math.sqrt(Math.pow(v.y, 2) + Math.pow(v.x, 2));
	        if (length != 0)
	        {
	            return new PairDouble(v.x/length, v.y/length);
	        }
	        return new PairDouble(0, 0);
	    }

}
