package utility;

import java.io.StringReader;
import java.io.StringWriter;
import java.text.DecimalFormat;

import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;


public class Trace {
	public long time;
	public double [] values = null;
	public int dim;
	public String type;
	
    public static String ACCELEROMETER = "accelerometer";
    public static String GYROSCOPE = "gyroscope";
    public static String MAGNETOMETER = "magnetometer";
    public static String ROTATION_MATRIX = "rotation_matrix";
    public static String GPS = "gps";
    public static String SPEED = "Vehicle Speed";
	

	public Trace() {
		time = 0;
		dim = 3;
		values = new double [dim];
	}
	
	
	public Trace(int d) {
		time = 0;
		dim = d;
		values = new double [dim];
	}
	public Trace(String type, int d) {
		this.type = type;
		time = 0;
		dim = d;
		values = new double [dim];
	}

	
	public void setValues(double x, double y, double z) {
		values[0] = x;
		values[1] = y;
		values[2] = z;
		dim = 3;
	}
	
	
	public void copyTrace(Trace trace) {
		this.time = trace.time;
		this.dim = trace.dim;
		this.type = trace.type;
		this.values = new double[dim];
		for(int i = 0; i < dim; ++i) {
			this.values[i] = trace.values[i];
		}
	}
	
	public void getTrace(String line) {
		
		String[] res = line.split(Constants.kInputSeperator);
		try {
			time = Long.parseLong(res[0]);
			//time = (long) (Double.parseDouble(res[0]) * 1000.0);
		} catch (NumberFormatException e) {
			//handle error time format
			time = 0;
		}
		//Log.log(dim, line);
		for(int i = 0; i < dim; ++i) {
			try {
				values[i] = Double.parseDouble(res[i + 1]);	
			} catch (NumberFormatException e) {
				values[i] = 0.0;
			}
		}
		
		//System.out.println(time + Constants.kSeperator + values[0]);
	}
	
	public String toString() {
		String res = new String("");
		res = res.concat(String.valueOf(time));
		for(int i = 0; i < dim; ++i) {
			res = res.concat(Constants.kOutputSeperator + String.valueOf(values[i]));
		}
		return res;
	}
	
	public String toJson() {	
		StringWriter sw = new StringWriter();
		JsonWriter writer = new JsonWriter(sw);
        try {
            writer.beginObject();
            writer.name("type").value(type);
            writer.name("time").value(time);
            writer.name("dim").value(dim);
            for (int i = 0; i < dim; ++i) {
                writer.name("x" + String.valueOf(i)).value(values[i]);
            }
            writer.endObject();
            writer.flush();
        } catch (Exception e) {
            Log.log(this, "convert to json error!!!!");
        }
        return sw.toString();
	}
	
	public void fromJson(String json) {
        StringReader sr = new StringReader(json);
		JsonReader reader = new JsonReader(sr);
		try {
            reader.beginObject();
            while (reader.hasNext()) {
                String name = reader.nextName();
                if (name.equals("type")) {
                    type = reader.nextString();
                } else if (name.equals("time")) {
                    time = reader.nextLong();
                } else if (name.equals("dim")) {
                    dim = reader.nextInt();
                    values = new double[dim];
                } else if (name.contains("x")) {
                    int index = Integer.valueOf(name.substring(1)).intValue();
                    values[index] = (float)reader.nextDouble();
                } else {
                    reader.skipValue();
                }
            }
            reader.endObject();
        } catch (Exception e) {
            Log.log(this, "parse from json error!!!!");
        }
		
	}

	
    public static double distance(Trace gps0, Trace gps1) {

        double lat1 = Math.toRadians(gps0.values[0]);
        double lng1 = Math.toRadians(gps0.values[1]);
        double lat2 = Math.toRadians(gps1.values[0]);
        double lng2 = Math.toRadians(gps1.values[1]);

        double p1 = Math.cos(lat1)*Math.cos(lat2)*Math.cos(lng1-lng2);
        double p2 = Math.sin(lat1)*Math.sin(lat2);

        double res = Math.acos(p1 + p2);
        if(res< Constants.kSmallEPSILON || res!=res) {
            res = 0.0;
        }
        //Log.log("dis:", res);
        return res * Constants.kEarthRadius;
    }
}