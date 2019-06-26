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

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "./data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
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
  double current_speed = 0; // mph
  double speed_inc = .4;
  int32_t time_step = 0;

  h.onMessage([&time_step, &speed_inc, &current_speed, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
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

          int prev_size = previous_path_x.size();
          double segment = 30.0;   // m
          double max_speed = 49.5; // mph

          // Initialize road map matrix with 3 lanes with 3 segments by 30 m for each lane
          // road_map[lane][car v][car s]
          double road_map[3][3][2] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

          time_step++;

          // Prediction
          for (int i = 0; i < sensor_fusion.size(); i++)
          {

            double ahead_car_s = sensor_fusion[i][5];
            double ahead_car_d = sensor_fusion[i][6];
            double ahead_car_vx = sensor_fusion[i][3];
            double ahead_car_vy = sensor_fusion[i][4];
            double ahead_car_speed = sqrt(ahead_car_vx * ahead_car_vx + ahead_car_vy * ahead_car_vy);
            int ahead_car_lane;

            //Check the lane # of the approaching car
            if (ahead_car_d > 8 && ahead_car_d < 12)
              ahead_car_lane = 2;
            else if (ahead_car_d > 4 && ahead_car_d < 8)
              ahead_car_lane = 1;
            else if (ahead_car_d < 4)
              ahead_car_lane = 0;

            double ahead_car_distance = ahead_car_s - car_s;

            // Fill the road map segments with cars v and s (90m ahead)
            if (ahead_car_d > 0)
            {
              if (ahead_car_distance > -5 && ahead_car_distance < segment)
              { //Add car v and s if car is in 30m
                road_map[ahead_car_lane][0][0] = ahead_car_speed * 2.24;
                road_map[ahead_car_lane][0][1] = ahead_car_s;
              }
              if (ahead_car_distance > segment && ahead_car_distance < 2 * segment)
              { //If there is a car in 30-60m add car v and s
                road_map[ahead_car_lane][1][0] = ahead_car_speed * 2.24;
                road_map[ahead_car_lane][1][1] = ahead_car_s;
              }
              if (ahead_car_distance > 2 * segment && ahead_car_distance < 3 * segment)
                road_map[ahead_car_lane][2][0] = ahead_car_speed * 2.24; //60 - 90 m
            }
          }

          // Behaviour Planning

          int new_lane = lane;
          double end_path_speed = 2.24 * (end_path_s - car_s) / (prev_size * 0.02); // calculating car speed at the end of previous path

          /*
          std::cout << "----------------------" << std::endl;
          for (int n = 0; n < 3; n++)
          {
            std::cout << "road[" << n << "]" << road_map[n][0][0] << " " << road_map[n][1][0] << " " << road_map[n][2][0] << std::endl;
          }
          std::cout << "----------------------" << std::endl;
          */

          //Checking the lane and planning the next action if the lane is not free
          if (road_map[lane][0][0] > 0)
          {

            if (lane == 0 || lane == 2)
            {
              if (road_map[1][0][0] == 0)
              {
                new_lane = 1;
              }
            }

            if (lane == 1)
            {
              if (road_map[0][0][0] == 0 && road_map[0][1][0] == 0)
                new_lane = 0; // If left lane is free and next segment 30-60m is also free
              else if (road_map[2][0][0] == 0 && road_map[2][1][0] == 0)
                new_lane = 2; // else if right lane is free and next segment 30-60m is also free
              else if (road_map[0][0][0] == 0)
                new_lane = 0; // else if left lane is free. Preference is set to go to left lane
              else if (road_map[2][0][0] == 0)
                new_lane = 2; // else go to right lane
            }

            //All lanes are occupied by vehicles. Decreasing car speed
            if (lane == new_lane)
            {

              if (end_path_speed > road_map[lane][0][0] && current_speed > road_map[lane][0][0])
              {

                current_speed -= speed_inc;
                //std::cout << "!!! Lanes are busy. Decreasing car speed: " << end_path_speed << " Curr:" << current_speed << "ahead car speed: " << road_map[lane][0][0] << std::endl;
              }

              // !!! Emergency break to avoid collision if the distance is less than 15m but our car speed is too fast
              if (road_map[lane][0][1] - car_s < 15 && current_speed > road_map[lane][0][0])
              {
                current_speed -= 5 * speed_inc;
                //std::cout << "!!! Emergency break. End path speed: " << end_path_speed << " Curr:" << current_speed << "car_s " << car_s << "ahead car speed: " << road_map[lane][0][0] << " ahead_s" << road_map[lane][0][1] << std::endl;
              }
            }
            else
              lane = new_lane;
          }
          // Lane is free. Increase speed if it's less than max. allowed speed
          else if (current_speed < max_speed && end_path_speed < max_speed)
          {
            current_speed += speed_inc;
            //std::cout << "Increase speed:  Curr:" << current_speed << std::endl;
          }

          vector<double> path_points_x;
          vector<double> path_points_y;

          //Refrence x,y, and yaw states
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // If previous path is empty using car x,y,yaw to create first points
          if (prev_size < 2)
          {

            //Use two points thats makes path tangent to the car
            double previous_car_x = car_x - cos(car_yaw);
            double previous_car_y = car_y - sin(car_yaw);

            path_points_x.push_back(previous_car_x);
            path_points_x.push_back(car_x);

            path_points_y.push_back(previous_car_y);
            path_points_y.push_back(car_y);
          }
          else
          {

            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];

            //Calculating reference yaw from 2 poins from previous path
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            path_points_x.push_back(ref_x_prev);
            path_points_x.push_back(ref_x);

            path_points_y.push_back(ref_y_prev);
            path_points_y.push_back(ref_y);
          }

          // Setting up points in 60m, 90m, 120m ahead according to the car lane
          vector<double> next_wp0 = getXY(car_s + 60, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 90, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 120, 2 + 4 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          path_points_x.push_back(next_wp0[0]);
          path_points_x.push_back(next_wp1[0]);
          path_points_x.push_back(next_wp2[0]);

          path_points_y.push_back(next_wp0[1]);
          path_points_y.push_back(next_wp1[1]);
          path_points_y.push_back(next_wp2[1]);

          // Switching coordinates to local car coordinates.
          for (int i = 0; i < path_points_x.size(); i++)
          {
            double shift_x = path_points_x[i] - ref_x;
            double shift_y = path_points_y[i] - ref_y;

            path_points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            path_points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          tk::spline s;
          s.set_points(path_points_x, path_points_y);

          //Start with previous path points
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate y and distance in 30 m ahead.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (int i = 1; i < 50 - prev_size; i++)
          {

            double N = target_dist / (0.02 * current_speed / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            //Rotate back to normal coordinates
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket if
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
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}