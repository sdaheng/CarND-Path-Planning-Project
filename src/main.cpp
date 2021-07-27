#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

bool ChangeLane(int lane, vector<vector<double> > car_track, vector<vector<double> > sensor_fusion, int path_size, double car_s, int lane_change){
	
	int lane_to_check = lane + lane_change;
	
	for(int i = 0 ; i < car_track[lane_to_check].size(); i ++) { //loop for 0,1 and 2 lane numbers
	
		double vx = sensor_fusion[car_track[lane_to_check][i]][3];
		double vy = sensor_fusion[car_track[lane_to_check][i]][4];
		double check_speed = sqrt((vx * vx) + (vy * vy));
		double check_car_s = sensor_fusion[car_track[lane_to_check][i]][5];
		
		check_car_s += ((double)path_size * 0.02 * check_speed); //if using previous points can project s value out
		//check if the car is in front of us and also if the gap is less than 30 or not
		
		if ((check_car_s < car_s) && ((car_s - check_car_s) < 10)) {
			return false;
		}
    
		if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
			return false;
		}
	}
	
	return true;
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  int lane = 1;
  double ref_vel = 2;

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          bool print_statements = false;
          int path_size = previous_path_x.size();

          if (path_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
			
          int number_of_lanes = 3; //0,1 and 2   we are concerned only with 1,2 and 3 
          vector<vector<double> > car_track(number_of_lanes); 
          
          for (int i = 0; i < sensor_fusion.size(); i ++) {
            float d = sensor_fusion[i][6];
            if (d >= 8) { //lane 3 car
              //car_track[2].push_back(sensor_fusion[i][0]);
              car_track[2].push_back(i);					
              if (print_statements){
                cout << "d>8 : " << d << endl;
              }
            } else if (d >= 4) { //lane 2 car
              //car_track[1].push_back(sensor_fusion[i][0]);
              car_track[1].push_back(i);
              if (print_statements){
              cout << "d>4 : " << d << endl;
              }
            } else if (d>0) {//left most lane
              //car_track[0].push_back(sensor_fusion[i][0]);
              car_track[0].push_back(i);
              if (print_statements){
                cout << "Else: " << d << endl;
              }
            }						
          }
			
          if (print_statements) {
            if (ref_vel < 8) {
              cout << "Sensor fusion size : " << sensor_fusion.size() << endl;
              cout << "car_track[1].size = " << car_track[0].size() << endl;
              cout << "car_track[2].size = " << car_track[1].size() << endl;
              cout << "car_track[3].size = " << car_track[2].size();
              cout << endl << endl;
            }
          }
			
          for (int i = 0 ; i < car_track[lane].size(); i++) { //loop for 0,1 and 2 lane numbers
          
            double vx = sensor_fusion[car_track[lane][i]][3];
            double vy = sensor_fusion[car_track[lane][i]][4];
            double check_speed = sqrt((vx * vx) + (vy * vy));
            double check_car_s = sensor_fusion[car_track[lane][i]][5];
            
            check_car_s += ((double)path_size*0.02*check_speed); //if using previous points can project s value out
            //check if the car is in front of us and also if the gap is less than 30 or not
            if ((check_car_s > car_s ) && ((check_car_s - car_s) < 30)) {
              //ref_vel = 29;
              too_close = true;
              break;
            }
          }
			
          int lane_change = 0;
          if (too_close) {	
            switch(lane) {
              
              //lane_change = lane_change +1 if shifting to right
              //lane_change = lane_change -1 if shifting to left
              case 0:
                if (ChangeLane(lane,car_track,sensor_fusion,path_size, car_s,lane_change+=1)) {
                  lane = 1;
                }
                break;
              case 1:
                if (ChangeLane(lane,car_track,sensor_fusion, path_size, car_s,lane_change-=1)) {
                  lane = 0;
                }else if (ChangeLane(lane,car_track,sensor_fusion,path_size, car_s,lane_change+=1)) {
                  lane = 2;
                }
                break;
              case 2:
                if (ChangeLane(lane,car_track,sensor_fusion, path_size, car_s,lane_change-=1)) {
                  lane = 1;
                }
                break;
              default:
                //Do nothing
                ;
            }
          }
			
          //These will contain spaced points that will be passed to spline to get a trajectory
          vector<double> x_points_for_spline;
          vector<double> y_points_for_spline;
          
          //Creating reference ponits
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          
          if (path_size < 2) {
            //Path is almost empty so create our own points using equations
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            x_points_for_spline.push_back(prev_car_x);
            x_points_for_spline.push_back(car_x);
            
            y_points_for_spline.push_back(prev_car_y);
            y_points_for_spline.push_back(car_y);
            
          } else {
            //if we have previous points then use them
            ref_x = previous_path_x[path_size - 1];
            ref_y = previous_path_y[path_size - 1];
            
            double ref_prev_car_x = previous_path_x[path_size-2];
            double ref_prev_car_y = previous_path_y[path_size-2];
            
            ref_yaw = atan2(ref_y - ref_prev_car_y, ref_x - ref_prev_car_x);
            
            x_points_for_spline.push_back(ref_prev_car_x);
            x_points_for_spline.push_back(ref_x);
            
            y_points_for_spline.push_back(ref_prev_car_y);
            y_points_for_spline.push_back(ref_y);
          }
          
          //Using 5 points in Frenet system for spline. Since we now have 2 starting points now getting the rest points.
          vector<double> spline_point3 = getXY(car_s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_point4 = getXY(car_s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_point5 = getXY(car_s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          x_points_for_spline.push_back(spline_point3[0]);
          x_points_for_spline.push_back(spline_point4[0]);
          x_points_for_spline.push_back(spline_point5[0]);

          y_points_for_spline.push_back(spline_point3[1]);
          y_points_for_spline.push_back(spline_point4[1]);
          y_points_for_spline.push_back(spline_point5[1]);
          
          
          //or use path_size
          for(int i = 0; i < x_points_for_spline.size(); i++){
            //shifting to car co-ordinate system
            double shift_x = x_points_for_spline[i] - ref_x;
            double shift_y = y_points_for_spline[i] - ref_y;
            
            x_points_for_spline[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            y_points_for_spline[i] = (shift_y * cos(0 - ref_yaw) + shift_x * sin(0 - ref_yaw));
          }
          
          tk::spline s;
          
          s.set_points(x_points_for_spline, y_points_for_spline);
          
          double dist_inc = 0.4;
          
          //or use path_size
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          //Break the spline points so that we do not overshoot the reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_distance = sqrt((target_x * target_x) + (target_y * target_y));
          
          double x_add_on = 0;
          
          //or use already computer path_size
          for (int i = 0; i < 50 - previous_path_x.size(); i++) {
            //0.224 -> this comes when we take acceleration as 5meter/second^2
            if (too_close) {
              ref_vel = ref_vel - 0.224;
            } else if (ref_vel < 49) {
              ref_vel = ref_vel + 0.224;
            }
            //divide by 2.24 because this equation is for m/s and not Miles Per Hour. ref_vel is set in Miles Per Hour
            double N = (target_distance / (0.02 * ref_vel / 2.24));
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;

            //transform back to global co-ordinate system
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (y_ref * cos(ref_yaw) + x_ref * sin(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout  <<  "Connected!!!"  <<  std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout  <<  "Disconnected"  <<  std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout  <<  "Listening to port "  <<  port  <<  std::endl;
  } else {
    std::cerr  <<  "Failed to listen to port"  <<  std::endl;
    return -1;
  }
  
  h.run();
}