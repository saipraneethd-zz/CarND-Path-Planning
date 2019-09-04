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
  double ref_vel = 0;
  int desired_lane  = 2; // 1,2,3

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_vel,&desired_lane]
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
          std::cout << "Car Speed : " << car_speed<< '\n';


          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          int path_size = previous_path_x.size();
          if (path_size > 0) {
            car_s = end_path_s;
          }
          //std::cout << path_size << '\n';
          // Lane identifiers for other cars
    			bool car_ahead = false;
    			bool car_to_left = false;
    			bool car_to_right = false;
          float veh_speed_ahead = 0;
    			// Find ref_v to use, see if car is in lane
    			for (int i = 0; i < sensor_fusion.size(); i++) {
    				// Car is in my lane
    				float d = sensor_fusion[i][6];
    				int car_lane;
    				if (d >= 0 && d < 4) {
    					car_lane = 1;
    				} else if (d >= 4 && d < 8) {
    					car_lane = 2;
    				} else if (d >= 8 && d <= 12) {
    					car_lane = 3;
    				}

    				// Check width of lane, in case cars are merging into our lane
    				double vx = sensor_fusion[i][3];
    				double vy = sensor_fusion[i][4];
    				double check_speed = sqrt(vx*vx + vy*vy);
    				double check_car_s = sensor_fusion[i][5];

    				// If using previous points can project an s value outwards in time
    				// (What position we will be in in the future)
    				// check s values greater than ours and s gap
    				//check_car_s += ((double)path_size*0.02*check_speed);

    				int gap = 30; // m

    				// Identify whether the car is ahead, to the left, or to the right
    				if (car_lane == desired_lane) {
    					// Car is ahead
    					car_ahead |= (check_car_s > car_s) && ((check_car_s - car_s) < gap);
              veh_speed_ahead = check_speed;
    				} else if (car_lane - desired_lane == 1) {
    					// Car is to the right
    					car_to_right |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
    				} else if (desired_lane - car_lane == 1) {
    					// Car is to the left
    					car_to_left |= ((car_s - gap) < check_car_s) && ((car_s + gap) > check_car_s);
    				}
    			}
          std::cout << "Car Ahead :" << car_ahead<< '\n';
          std::cout << "Car Left :" << car_to_left<< '\n';
          std::cout << "Car Right :" << car_to_right<< '\n';
    			// Modulate the speed to avoid collisions. Change lanes if it is safe to do so (nobody to the side)
    			double acc = 0.224;
    			double max_speed = 49.5;
          if(!car_to_right){
            std::cout << "Undi" << '\n';
          }
    			if (car_ahead) {
    				// Either shift lanes or slow down
    				if (!car_to_right && desired_lane < 3) {

    					desired_lane++; // Shift right lane
    				} else if (!car_to_left && desired_lane > 1) {
    					desired_lane--; // Shift left lane
    				} else {
    					// Slow down ahead
              std::cout << "slow " << '\n';
              if(ref_vel > veh_speed_ahead){
    					       ref_vel -= acc;
              }
    				}
    			} else {
    				// if (desired_lane  != 2) {
    				// 	// Ego lane - Center
    				// 	if ((desired_lane == 3 && !car_to_left) || (desired_lane == 1 && !car_to_right)) {
    				// 		desired_lane = 2;
    				// 	}
    				// }
    				if (ref_vel < max_speed) {
    					// No car ahead AND we are below the speed limit -> speed limit
    					ref_vel += acc;
    				}
    			}
          std::cout<<"Reference Velocity : " << ref_vel << '\n';
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
         vector<double> ptsx, ptsy;
         double pos_x,pos_y,pos_x2,pos_y2, angle,pos_s,pos_d;
         // Reference x, y, yaw states
   			 double ref_x = car_x;
   			 double ref_y = car_y;
   			 double ref_yaw = deg2rad(car_yaw);


         if (path_size < 2) {
           pos_x2 = ref_x - cos(car_yaw);
           pos_y2 = ref_y - sin(car_yaw);
           ptsx.push_back(pos_x2);
           ptsx.push_back(ref_x);
           ptsy.push_back(pos_y2);
           ptsy.push_back(ref_y);
         } else {
           ref_x = previous_path_x[path_size-1];
           ref_y = previous_path_y[path_size-1];

           pos_x2 = previous_path_x[path_size-2];
           pos_y2 = previous_path_y[path_size-2];
           ref_yaw = atan2(ref_y-pos_y2,ref_x-pos_x2);
           ptsx.push_back(pos_x2);
           ptsx.push_back(ref_x);
           ptsy.push_back(pos_y2);
           ptsy.push_back(ref_y);
         }

         vector<double> xy1 =getXY(car_s + 30,2+((desired_lane-1)*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
         vector<double> xy2 =getXY(car_s + 60,2+((desired_lane-1)*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
         vector<double> xy3 =getXY(car_s + 90,2+((desired_lane-1)*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
         ptsx.push_back(xy1[0]);
         ptsx.push_back(xy2[0]);
         ptsx.push_back(xy3[0]);
         ptsy.push_back(xy1[1]);
         ptsy.push_back(xy2[1]);
         ptsy.push_back(xy3[1]);
         // Shift coordinate system with origin as Car
         for (int i = 0; i < ptsx.size(); i++) {
           double shift_x = ptsx[i] - ref_x;
           double shift_y = ptsy[i] - ref_y;
           ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
           ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
         }
         tk::spline s;
         s.set_points(ptsx,ptsy);

         for (int i = 0; i < path_size; ++i) {
           next_x_vals.push_back(previous_path_x[i]);
           next_y_vals.push_back(previous_path_y[i]);
         }
         double target_x = 30.0;
   			 double target_y = s(target_x);
   			 double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
   			 double x_val = 0;
         double y_val = 0;
         for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
   				double N = (target_dist/(.02*ref_vel/2.24));
   				x_val = x_val + (target_x) / N;
   				y_val = s(x_val);
   				// Rotate back to normal after rotating it earlier
   				double x_point = (x_val * cos(ref_yaw) - y_val*sin(ref_yaw));
   				double y_point = (x_val * sin(ref_yaw) + y_val*cos(ref_yaw));

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
