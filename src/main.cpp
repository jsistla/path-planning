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
#include "constants.h"


/* #################### NAMESPACES #################### */
using namespace std;
using spline = tk::spline;
using json = nlohmann::json;

/* #################### SIMPLIFICATION #################### */
typedef vector<int> vi_t;
typedef vector<double> vd_t;
typedef vector<vector<int>> vvi_t;
typedef vector<vector<double>> vvd_t;
#define pb push_back

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double ref_vel = 0.0;

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

// find the lane with the given value of d
int find_lane (double value)
{
  int current_lane = -1;
  if ((value > 0 ) && (value < LANE_WIDTH ))
    current_lane = 0;
  else if ((value > LANE_WIDTH ) && (value < 2*LANE_WIDTH ))
    current_lane = 1;
  else if((value > 2*LANE_WIDTH ) && (value < 3*LANE_WIDTH ))
    current_lane = 2;

return current_lane;
}

int ClosestWaypoint(double x, double y, vd_t maps_x, vd_t maps_y)
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

int NextWaypoint(double x, double y, double theta, vd_t maps_x, vd_t maps_y)
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
vd_t getFrenet(double x, double y, double theta, vd_t maps_x, vd_t maps_y)
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

  // caLANE_CENTERulate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }
  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vd_t getXY(double s, double d, vd_t maps_s, vd_t maps_x, vd_t maps_y)
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

// Check for cars ahead in the same lane

vd_t  check_lane_forward_behaviour(vector<vector<double >> sensor_fusion, int lane, vd_t params,unsigned int pp_size, unsigned int off ) {

  double status = 1;
  int current_lane = find_lane(params[3]); // get actual car lane
  vd_t id_ahead = {0,0}, dist_ahead= {LARGE_CONSTANT,LARGE_CONSTANT};
  double nc_id;
  for (unsigned int k = 0; k < sensor_fusion.size();++k) {
    vd_t  nc = sensor_fusion[k];
    if (nc[6] > (LANE_CENTER+LANE_WIDTH*lane - 2) && (nc[6] < (LANE_CENTER+LANE_WIDTH*lane +2) ) ) {   // if its in the same lane
      double nc_speed = sqrt(nc[3] *nc[3] + nc[4]*nc[4]);  // next car's speed
      double nc_next_s = nc[5] + (pp_size +1)*SIM_TICK*nc_speed;   //cars_next location
      double own_s = (pp_size > 0)? params[6]: params[2] ;
      if( ( nc_next_s > own_s) && ((nc_next_s - own_s) <  LANE_HORIZON + off )){  // Find the closest car in horizon, off value to avoid unnecessary overtaking
        status = 0;
        if (dist_ahead[0] == LARGE_CONSTANT) { // only first time
          dist_ahead[0] = nc_next_s - own_s;
          id_ahead[0] = nc[0];
        } else {
          dist_ahead[1] = nc_next_s - own_s ;
          id_ahead[1] = nc[0];
          // Check for the closest car, if multiple cars in range
          if ( MIN(dist_ahead[1],dist_ahead[0]) == dist_ahead[1]) {
            dist_ahead[0] = dist_ahead[1];
            id_ahead[0] = id_ahead[1];
          }
          nc_id=k;
        }
      } // end of horizon check
    } // end of loop for cars in same lane
  } // end of for loop for checking all cars
  return {status,id_ahead[0],dist_ahead[0]};   // false if car ahead, otherwise true
}

// check for cars behind in the same lane
vd_t  check_lane_back_behaviour(vector<vector<double >> sensor_fusion, int lane, vd_t params,unsigned int pp_size ) {

  double status =1;
  int clane = find_lane(params[3]); // get actual car lane
  vd_t id_ahead = {0,0}, dist_ahead= {LARGE_CONSTANT,LARGE_CONSTANT}, speed_bck= {LARGE_CONSTANT,LARGE_CONSTANT};

  for (unsigned int k = 0; k < sensor_fusion.size();++k){
    vd_t  nc = sensor_fusion[k];
    int nc_id;
    if (nc[6] > (LANE_CENTER+LANE_WIDTH*lane - 2) && (nc[6] < (LANE_CENTER+LANE_WIDTH*lane +2) ) ){   // if its in the same lane
      double nc_speed = sqrt(nc[3] *nc[3] + nc[4]*nc[4]);  // next car's speed
      double nc_next_s = nc[5] + (pp_size +1)*SIM_TICK*nc_speed;   //cars_next location
      double own_s = (pp_size > 0)? params[6]: params[2] ;

      if( ( nc_next_s < own_s) && (( own_s -nc_next_s) <  LANE_HORIZON  )){  // Find the closest car in horizon
        status = 0;
        if (dist_ahead[0] == LARGE_CONSTANT) { // only first time
          dist_ahead[0] = own_s - nc_next_s;
          speed_bck[0] = nc_speed;
          id_ahead[0] = nc[0];
        } else {
          dist_ahead[1] = own_s - nc_next_s;
          speed_bck[1] = nc_speed;
          id_ahead[1] = nc[0];
          // Check for the closest car, if multiple cars in range
          if ( MIN(dist_ahead[1],dist_ahead[0]) == dist_ahead[1]) {
            dist_ahead[0] = dist_ahead[1];
            id_ahead[0] = id_ahead[1];
            speed_bck[0] = speed_bck[1];
          }
        }
      } // end of horizon check
    } // end of loop for cars in same lane
  } // end of for loop for checking all cars

  return {status, id_ahead[0],dist_ahead[0], speed_bck[0]} ;   // false if car ahead, otherwise true
}

// calculate cost of path change
double fn_calc_cost (double inp) {
  double val = 1 - exp(-1/inp);
  return val;
}

// evaluate the cost of changing lanes from current lane
vd_t check_costs(vvd_t sensor_fusion, int lane, vd_t params, unsigned int pp_size ){


  double new_lane = 0, cost1= MAX_COST , cost2 = MAX_COST;
  if (lane == 0){  // if car is originally in left most lane, check only for right lane
    new_lane = lane +1;
    auto fwd_1 = check_lane_forward_behaviour(sensor_fusion, new_lane,  params, pp_size, LANE_OFF );
    cost1 =fn_calc_cost(fwd_1[2]);  // get fwd distance cost
    auto bck_1 = check_lane_back_behaviour(sensor_fusion, new_lane, params, pp_size );
    cost1 +=fn_calc_cost(bck_1[2]);  // add backward distance cost
  }

  if (lane == 1) { // if car is in middle, check for both lanes
     auto fwd_1 = check_lane_forward_behaviour(sensor_fusion, lane + 1,  params, pp_size, LANE_OFF );
     cost1 =fn_calc_cost(fwd_1[2]);
     auto bck_1 = check_lane_back_behaviour(sensor_fusion, lane + 1, params, pp_size );
     cost1 +=fn_calc_cost(bck_1[2]);
     auto fwd_2 = check_lane_forward_behaviour(sensor_fusion, lane - 1,  params, pp_size, LANE_OFF );
     cost2 =fn_calc_cost(fwd_2[2]);
     auto bck_2 = check_lane_back_behaviour(sensor_fusion, lane - 1, params, pp_size );
     cost2 +=fn_calc_cost(bck_2[2]);
     if (cost1 <= cost2) {
       new_lane = lane + 1;
     } else {
       new_lane = lane - 1;
       cost1 = cost2;
     }
  }

  if (lane == 2){ // car on the right, check for middle lane
      new_lane = lane - 1;
      auto fwd_1 = check_lane_forward_behaviour(sensor_fusion, new_lane ,  params, pp_size, LANE_OFF );
      cost1 =fn_calc_cost(fwd_1[2]);
      auto bck_1 = check_lane_back_behaviour(sensor_fusion, new_lane , params, pp_size );
      cost1 +=fn_calc_cost(bck_1[2]);
    }

  return {new_lane,cost1};

}

//convert global XY coordinates to car's reference frame to allow for polynomial smoothing
void global_XY_car_ref( vd_t &ptx , vd_t &pty, double x, double y, double yaw)
{
  for (int k = 0; k < ptx.size(); ++k){
    double x_origin = ptx[k] - x;
    double y_origin = pty[k] - y;

    ptx[k] = x_origin * cos(-yaw) - y_origin*sin(-yaw);
    pty[k] = x_origin * sin( -yaw) + y_origin*cos(-yaw);

    }
}

// get next path points based upon spline
void get_next_pts(vd_t &x_pts, vd_t& y_pts, vd_t & next_x, vd_t &next_y,\
    tk::spline s, unsigned int pp_size, double x, double y, double yaw, double & velocity)
{

  double x_horizon = PATH_HORIZON ;  // horizon
  double y_horizon = s(x_horizon);
  double path = sqrt((x_horizon*x_horizon)  + (y_horizon*y_horizon));
  double x_addon = 0;

  for(int k = 1; k < MAX_POINTS - pp_size; ++k) {
    double N = 2.24* path / (SIM_TICK * velocity);
    double x_point = x_addon + (x_horizon/N);
    double y_point = s(x_point);

    x_addon = x_point ;

    double x_glo_cors = x_point *cos(yaw) - y_point*sin(yaw);
    double y_glo_cors = x_point *sin(yaw) + y_point*cos(yaw);

    x_glo_cors += x;
    y_glo_cors += y;

    next_x.pb(x_glo_cors);
    next_y.pb(y_glo_cors);

    }
}

int main() {

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vd_t map_waypoints_x;
  vd_t map_waypoints_y;
  vd_t map_waypoints_s;
  vd_t map_waypoints_dx;
  vd_t map_waypoints_dy;

  string map_file_ = WP_FILE;
  double max_s = MAX_S_VALUE;

  //set maximum speed, acceleration, and jerk
  const double speed_limit = MAX_SPEED;
  const double a_max = MAX_ACCEL;
  const double j_max = MAX_JERK;

  //desired lane
  int target_lane = 1;

  //vector defining Frenet d coordinate for center of each lane
  vd_t lane_positions;
  lane_positions.pb (2.0);
  lane_positions.pb (6.0);
  lane_positions.pb (10.0);

  //keep track of car's acceleration
  double car_a = 0.0;
  double set_speed = 0.0;

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
    map_waypoints_x.pb(x);
    map_waypoints_y.pb(y);
    map_waypoints_s.pb(s);
    map_waypoints_dx.pb(d_x);
    map_waypoints_dy.pb(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &speed_limit, &a_max, &j_max, &car_a, &set_speed, &lane_positions, &target_lane, &max_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
           vd_t params = {car_x,car_y,car_s,car_d,car_yaw,car_speed,end_path_s, end_path_d };
           // Sensor Fusion Data, a list of all other cars on the same side of the road.
           auto sensor_fusion = j[1]["sensor_fusion"];

           json msgJson;
           unsigned int pp_size = previous_path_x.size();   //previous path size

           int lane = find_lane(car_d);
           vd_t next_x_vals;
           vd_t next_y_vals;

           // put previous path points in next points' vector
           for(int k = 0; k < previous_path_x.size(); ++k) {
             next_x_vals.pb(previous_path_x[k]);
             next_y_vals.pb(previous_path_y[k]);
           }

           //  define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
           vd_t x_pts;
           vd_t y_pts;

           double x_ref = car_x;
           double y_ref = car_y;
           double yaw_ref = deg2rad(car_yaw);


           // Check if keep going straight is fine
           auto result = check_lane_forward_behaviour(sensor_fusion, lane, params,pp_size,0);

           if (0 == result[0]) { // car ahead, react by changing lane and/or reduce speed
             // check costs for moving to appropriate lane
             auto lane_cost= check_costs(sensor_fusion, lane, params, pp_size);
             // check costs for staying in the same lane
             auto cost1 = fn_calc_cost(double(result[2]));

             if (   (cost1 > lane_cost[1])      && (abs(cost1 - lane_cost[1]) > THRESH)   ){
               lane = lane_cost[0];   // lane change
               ref_vel += 2.0/2.24;
             }

             ref_vel -= 2.0/2.24;  // reduce velocity as theres a car ahead or lane changed
         } else if (ref_vel < REF_VEL) {  // keep driving straight and increase speed
             ref_vel += 2.0/2.24;
             if (ref_vel >  REF_VEL)	// safety check on max speed
               ref_vel = REF_VEL;
         }

         // get future trajectory
         if (pp_size < 2) { // simulation just started
           x_pts.pb(car_x - cos(car_yaw));
           x_pts.pb(car_x);
           y_pts.pb(car_y - sin(car_yaw));
           y_pts.pb(car_y);
         } else {  // car has a previous path

           x_ref = previous_path_x[pp_size - 1];
           y_ref = previous_path_y[pp_size - 1];

           double x_ref_prev = previous_path_x[pp_size - 2];
           double y_ref_prev = previous_path_y[pp_size - 2];

           x_pts.pb(x_ref_prev);
           x_pts.pb(x_ref);

           y_pts.pb(y_ref_prev);
           y_pts.pb(y_ref);

           yaw_ref = atan2(y_ref - y_ref_prev,x_ref-x_ref_prev);

         }

         cout << x_pts[0] << ", " << y_pts[0] << ", " << x_pts[1] << ", " << y_pts[1] << ", " << car_yaw << "\n";

         vd_t wp0 = getXY(car_s + 60,(LANE_CENTER + LANE_WIDTH* lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
         vd_t wp1 = getXY(car_s + 75,(LANE_CENTER + LANE_WIDTH* lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
         vd_t wp2 = getXY(car_s + 90,(LANE_CENTER + LANE_WIDTH*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

         x_pts.pb(wp0[0]);
         x_pts.pb(wp1[0]);
         x_pts.pb(wp2[0]);

         y_pts.pb(wp0[1]);
         y_pts.pb(wp1[1]);
         y_pts.pb(wp2[1]);


          // change to car coordinates
          global_XY_car_ref(x_pts , y_pts, x_ref, y_ref, yaw_ref );

          tk::spline s;  //declare spline
          s.set_points(x_pts,y_pts); // spline for 5 points
          // get future points, code based on Project Q&A video
          get_next_pts(x_pts,y_pts,next_x_vals, next_y_vals, s, pp_size,x_ref,y_ref, yaw_ref,ref_vel);
          // Finished
          // Sending points to the simulator
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
