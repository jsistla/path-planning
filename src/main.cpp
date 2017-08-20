#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <stdlib.h>

/* #################### NAMESPACES #################### */
using namespace std;
using spline = tk::spline;
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double ref_val = 0.0;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;

  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{

  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}


//function to take car's location and calculate bend radius of road ahead
double curve_radius(double car_x, double car_y, vector<double> maps_x, vector<double> maps_y) {

    //find closest waypoint, use that plus the nest two points in calculating radius
    int p_1 = ClosestWaypoint(car_x,car_y,maps_x,maps_y);

    int num_waypoints = maps_x.size();

    int p_2 = p_1 + 1;

    if (p_2 >= num_waypoints)
        p_2 = p_2 - num_waypoints;

    int p_3 = p_1 + 2;

    if (p_3 >= num_waypoints)
        p_3 = p_3 - num_waypoints;

    //calculate road radius using formula for radius of circumscribing circle of a triangle
    double a = distance(maps_x[p_1], maps_y[p_1], maps_x[p_2], maps_y[p_2]);
    double b = distance(maps_x[p_2], maps_y[p_2], maps_x[p_3], maps_y[p_3]);
    double c = distance(maps_x[p_3], maps_y[p_3], maps_x[p_1], maps_y[p_1]);

    double s = (a+b+c)/2;

    double area = sqrt(s*(s-a)*(s-b)*(s-c));
    //return value of infinity for effectively straight road
    double radius;
    if (area < 0.001)
        radius = std::numeric_limits<double>::infinity();
    else
        radius = (a*b*c)/(4*a);

    return radius;

}


//function to determine best lane to be in
int choose_lane(double car_s, double car_d, double max_speed, vector<vector<double>> sensor_fusion, vector<double> lane_positions, int current_lane, int prev_points, double max_s)
{
    //
    vector<bool> safe;
    vector<double> traffic_speed;
    vector<double> clearance_ahead;
    vector<double> clearance_behind;

    int chosen_lane = current_lane;

    for (int j=0; j<lane_positions.size(); j++)
    {
        safe[j] = true;
        traffic_speed.push_back(100.0);
        clearance_ahead.push_back(1000.0);
        clearance_behind.push_back(1000.0);
        for(int i=0; i<sensor_fusion.size(); i++)
            {
                //determine if car is directly ahead
                double sensor_d = sensor_fusion[i][6];
                if (abs(sensor_d - lane_positions[j]) < 2.7)
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];

                    check_car_s += (double)prev_points*.02*check_speed;

                    double check_distance = check_car_s-car_s;

                    //adjust check_distance when wrapping around max_s to zero
                    if (check_distance > (max_s/2.0))
                        check_distance -= max_s;
                    if (check_distance < (-1.0*max_s/2.0))
                        check_distance += max_s;

                    if (check_distance > 0 && check_speed < traffic_speed[j])
                        traffic_speed[j] = check_speed;

                    if( check_distance > 0 && abs(check_distance) < clearance_ahead[j])
                        clearance_ahead[j] = check_distance;

                    if( check_distance < 0 && abs(check_distance) < clearance_behind[j])
                        clearance_behind[j] = check_distance;

                    if( abs(check_distance) < 20)
                        safe[j] = false;
                }
            }

    }


    //look at lane to the left unless car is in leftmost lane
    if (current_lane > 0)
    {
        if (safe[current_lane-1] && (traffic_speed[current_lane-1] > max_speed))
        {
            chosen_lane = current_lane-1;
            max_speed = traffic_speed[current_lane-1];
        }
    }

    //look at lane to the right unless car is in rightmost lane
    if (current_lane < lane_positions.size())
    {
        if (safe[current_lane+1] && (traffic_speed[current_lane+1] > max_speed))
        {
            chosen_lane = current_lane+1;
            max_speed = traffic_speed[current_lane-1];
        }
    }

    return chosen_lane;


}


int main() {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  //desired lane
  int target_lane = 1;
  int prev_target_lane = 1;

  //vector defining Frenet d coordinate for center of each lane
  vector<double> lane_positions;
  lane_positions.push_back (2.0);
  lane_positions.push_back (6.0);
  lane_positions.push_back (10.0);

  //sex maximum speed, acceleration, and jerk

  const double speed_limit = 22.2;

  const double a_max = 9.5;

  const double j_max = 45.0;

  //keep track of car's acceleration
  double car_a = 0.0;

  double set_speed = 0.0;

  //double set_a = 0.0;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &speed_limit, &a_max, &j_max, &car_a, &set_speed, &lane_positions, &target_lane, &prev_target_lane, &max_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
           // j[1] is the data JSON object
           // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /*for(int i = 0; i < 50; i++) {
            double next_s = car_s + (i+1)*dist_inc;
            double next_d = 6; //constant per lane width
            vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }*/
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          //*********************************************************************************




          bool too_close = false;

          int prev_points = previous_path_x.size();

	  if(prev_points > 0)
		  car_s = end_path_s;


            //iterate through detected vehicles, determine if there is a car ahead going below speed limit
            double check_distance = 1000;
            double check_speed = 1000;

            for(int i=0; i<sensor_fusion.size(); i++)
            {
                //determine if car is directly ahead
                float sensor_d = sensor_fusion[i][6];
                if ((sensor_d < (2+4*target_lane+2)) && (sensor_d > (2+4*target_lane-2)))
                {
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    check_speed = sqrt(vx*vx + vy*vy);
                    double check_car_s = sensor_fusion[i][5];

                    check_car_s += (double)prev_points*.02*check_speed;

                    check_distance = check_car_s-car_s;

                    //adjust check_distance when wrapping around max_s to zero
                    /*if (check_distance > (max_s/2.0))
                        check_distance -= max_s;
                    if (check_distance < (-1.0*max_s/2.0))
                        check_distance += max_s;*/

                    if( (check_car_s > car_s) && (check_distance<35) ) {
                        too_close = true;
		    }
                }
            }



            if(too_close)
            {
                ref_val -= 0.224;
            }
            else if (ref_val < 49.50)
            {
                ref_val += 0.224;
            }

            //if (check_speed < speed_limit*0.9)
                //target_lane = choose_lane(car_s, car_d, set_speed, sensor_fusion, lane_positions, target_lane, prev_points, max_s);




            //if target speed or lane has changed, ignore previous path and recalculate from scratch


            //if(prev_target_lane != target_lane || abs(car_speed - set_speed) > 0.2)
            //    prev_points = 0;


          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_points < 2)
            {
                //create two points defining a path tangent to the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            //use end of previous path as starting reference
            else
            {
                ref_x = previous_path_x[prev_points-1];
                ref_y = previous_path_y[prev_points-1];

                double ref_x_prev = previous_path_x[prev_points-2];
                double ref_y_prev = previous_path_y[prev_points-2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);


                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }



            cout << ptsx[0] << ", " << ptsy[0] << ", " << ptsx[1] << ", " << ptsy[1] << ", " << car_yaw << "\n";
            cout << check_distance << "\n";

            //define rough path in Frenet coordinates, convert to XY

            vector<double> next_wp0 = getXY(car_s+30, lane_positions[target_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, lane_positions[target_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, lane_positions[target_lane], map_waypoints_s, map_waypoints_x, map_waypoints_y);


            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);


            //convert global XY coordinates to car's reference frame to allow for polynomial smoothing
            for (int i=0; i<ptsx.size(); i++)
            {
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;

                ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }


            //create spline for smooth path
            tk::spline sp;
            sp.set_points(ptsx,ptsy);

            //load previous points into path planner
            for (int i=0; i<prev_points; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            //calculate remaining points for path planner based on spline
            double target_x = 30.0;
            double target_y = sp(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            for(int i = 0; i <= 50-prev_points; i++)
            {
                double N = (target_dist/(.02*ref_val/2.24));
                double x_point = x_add_on+target_x/N;
                double y_point = sp(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                //convert points back to global coordinates
                x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

            //double radius = curve_radius(car_x, car_y, map_waypoints_x, map_waypoints_y);

            prev_target_lane = target_lane;

          //*********************************************************************************
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
