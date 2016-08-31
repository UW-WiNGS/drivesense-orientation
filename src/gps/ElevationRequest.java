package gps;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UnsupportedEncodingException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLEncoder;
import java.util.ArrayList;
import java.util.List;

import utility.Trace;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public class ElevationRequest {

	private static String fpath = "/home/lkang/Dropbox/projects/obd/data/bozhao/sameRoute/bozhao/";
	/**
	 * 2,500 requests per 24 hour period. 512 locations per request. 25,000
	 * total locations per 24 hour period. 10 requests per second.
	 */

	private final static String API_KEY_lei = "AIzaSyAT2KGLs2smYp1Ce42xNrw8xd6GBjaioJ4";
	private final static String API_KEY_bozhao = "AIzaSyD7EDp31VN01zkyMOSLrvkb5ehfobXOTxI";
	private final static String API_KEY = API_KEY_lei;

	public final static int maxLocationPerRequest = 20;



	public static String excutePost(String targetURL, String urlParameters) {
		URL url;
		HttpURLConnection connection = null;

		try {
			url = new URL(targetURL);
			connection = (HttpURLConnection) url.openConnection();
			connection.setRequestMethod("POST");
			connection.setRequestProperty("Content-Type",
					"application/x-www-form-urlencoded");
			connection.setRequestProperty("Content-Length",
					"" + Integer.toString(urlParameters.getBytes().length));
			connection.setRequestProperty("Content-Language", "en-US");

			connection.setUseCaches(false);
			connection.setDoInput(true);
			connection.setDoOutput(true);

			// Send request
			DataOutputStream wr = new DataOutputStream(
					connection.getOutputStream());
			wr.writeBytes(urlParameters);
			wr.flush();
			wr.close();

			// Get Response
			InputStream is = connection.getInputStream();
			BufferedReader rd = new BufferedReader(new InputStreamReader(is));
			String line;
			StringBuffer response = new StringBuffer();
			while ((line = rd.readLine()) != null) {
				response.append(line);
				response.append('\r');
			}
			rd.close();
			return response.toString();
		} catch (Exception e) {
			e.printStackTrace();
			return null;
		} finally {
			if (connection != null) {
				connection.disconnect();
			}
		}
	}


	public static double getAltitude(double lat, double lng) {
		double res = 0.0;
		String urlParameters = null;
		try {
			urlParameters = "fName=" + URLEncoder.encode("???", "UTF-8")
					+ "&lName=" + URLEncoder.encode("???", "UTF-8");
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		String lnglat = String.valueOf(lat) + ","
				+ String.valueOf(lng);
		String url = "https://maps.googleapis.com/maps/api/elevation/json?locations="
				+ lnglat + "&key=" + API_KEY;
		String str = excutePost(url, urlParameters);
		JsonObject newObj = new JsonParser().parse(str).getAsJsonObject();
		res = newObj.get("results").getAsJsonArray().get(0).getAsJsonObject()
				.get("elevation").getAsDouble();
		return res;
	}
	
	private static String constructURL(List<Trace> gps) {
		String lnglat = "";
		for (int i = 0; i < gps.size(); i++) {
			lnglat = lnglat + String.valueOf(gps.get(i).values[0]) + ","
					+ String.valueOf(gps.get(i).values[1]) + "|";
		}
		lnglat = lnglat.substring(0, lnglat.length() - 1); // removes last |
		String url = "https://maps.googleapis.com/maps/api/elevation/json?locations="
				+ lnglat + "&key=" + API_KEY;
		return url;

	}
	

	public static List<Trace> requestElevations(List<Trace> gps) {
		List<Trace> locs = new ArrayList<Trace>();
		assert gps.size() <= maxLocationPerRequest;
		String urlParameters = null;
		String url = null;
		try {
			urlParameters = "fName=" + URLEncoder.encode("???", "UTF-8")
					+ "&lName=" + URLEncoder.encode("???", "UTF-8");
			url = constructURL(gps);
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		String str = excutePost(url, urlParameters);
		//Log.log(str);

		JsonObject newObj = new JsonParser().parse(str).getAsJsonObject();
		JsonArray newArray = newObj.get("results").getAsJsonArray();

		//Log.log(newArray.size());

		for (int i = 0; i < newArray.size(); i++) {
			JsonElement res = newArray.get(i);
			Trace trace = new Trace(3);
			trace.copyTrace(gps.get(i));
			trace.values[2] = res.getAsJsonObject().get("elevation").getAsDouble();
			locs.add(trace);
		}
		return locs;
	}

}
