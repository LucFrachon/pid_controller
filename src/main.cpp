#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "twiddle.cpp"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


//Reset simulator during tuning phase
void reset_simulator(uWS::WebSocket<uWS::SERVER>& ws){
std::string msg("42[\"reset\",{}]");
ws.send(msg.data(),msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  PID steer_pid, speed_pid;
  
  double Kp, Ki, Kd;
  double dKp, dKi, dKd;
  
  Kp = 0.189477; Ki = 9.47387e-07; Kd = 1.88609;  //Good for 40mph, comment out when tuning twiddle
  //Kp = 0.20; Ki = 1.e-5; Kd = 2.0;  //Initial values, uncomment when tuning twiddle
  dKp = 0.02, dKi = 1.e-6, dKd = 0.2;

  steer_pid.Init(Kp, Ki, Kd);
  speed_pid.Init(0.15, 0., 0.);

  Twiddle tw;
  tw.Init(Kp, Ki, Kd, dKp, dKi, dKd, 0.2, 1000, 30);
  tw.tuning = false;  // Set to true to tune PID parameters


  const double target_speed = 40.;


  h.onMessage([&steer_pid, &speed_pid, &target_speed, &tw](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
          
        if (event == "telemetry") {

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          if (tw.tuning) 
          {
            if ((tw.step == 1) && (tw.directions[tw.p_i] == 1))
            {
              tw.p[tw.p_i] += tw.dp[tw.p_i];
            }

            //std::cout << "Tuning\nStep: " << tw.step << std::endl;
            tw.sum_err += cte * cte;

            if ((tw.step == tw.max_steps || std::fabs(cte) > 2.2) && tw.sumScaledDp() > tw.tolerance
              && tw.epoch <= tw.max_epochs) //Max steps reached, vehicle left road or max epochs since last improved not reached
            {
              
              std::cout << "\nParameter index: " << tw.p_i;
              std::cout << "\nInitial p: " << tw.p[tw.p_i] << "\tInitial dp: " << tw.dp[tw.p_i];
              tw.avg_err = tw.sum_err / tw.max_steps;
              std::cout  << "\nAverage error: " << tw.avg_err << "\tBest error: " << tw.best_err;
              std::cout << "\nInitial direction for p: " << tw.directions[tw.p_i];

              if (tw.avg_err < tw.best_err)  //if improved error
              {
                tw.best_err = tw.avg_err;
                tw.best_p[0] = tw.p[0];
                tw.best_p[1] = tw.p[1];
                tw.best_p[2] = tw.p[2];
                tw.epoch = 0;
                tw.directions[tw.p_i] = 1;
                std::cout << "\nNew direction for p: " << tw.directions[tw.p_i];

                tw.updateDp(1 + tw.dp_factor);  //increase dp by x1.1
                std::cout << "\nNew dp: " << tw.dp[tw.p_i];
                tw.p[tw.p_i] += tw.dp[tw.p_i];
                std::cout << "\nNew p: " << tw.p[tw.p_i];
                std::cout << "\nSum of dp: " << tw.sumScaledDp() << std::endl;
                
                tw.p_i = (tw.p_i + 1) % 3;
              }
              else if (1 == tw.directions[tw.p_i])
              {
                tw.p[tw.p_i] -= 2 * tw.dp[tw.p_i];  //decrease p[i] twice because we increased it once before
                std::cout << "\nNew p: " << tw.p[tw.p_i];
                tw.directions[tw.p_i] = -1;
                std::cout << "\nNew direction for p: " << tw.directions[tw.p_i];
              }
              else  // we were going down  
              {
                tw.p[tw.p_i] += tw.dp[tw.p_i];  //increase p[i] to get to the last best value
                std::cout << "\nNew p: " << tw.p[tw.p_i];
                tw.updateDp(1 - tw.dp_factor);  //decrease dp by x0.9
                std::cout << "\nNew dp: " << tw.dp[tw.p_i];
                std::cout << "\nSum of dp: " << tw.sumScaledDp() << std::endl;
                tw.directions[tw.p_i] = 1;
                std::cout << "\nNew direction for p: " << tw.directions[tw.p_i];
                tw.p_i = (tw.p_i + 1) % 3;
              }
              
              std::cout << "\nCurrent parameters:\n"
                << tw.p[0] << "\t" << tw.p[1] << "\t" << tw.p[2] << std::endl;
              std::cout << "\nCurrent best parameters:\n"
                << tw.best_p[0] << "\t" << tw.best_p[1] << "\t" << tw.best_p[2] << std::endl << std::endl;

              
              steer_pid.Kp = tw.p[0];
              steer_pid.Ki = tw.p[1];
              steer_pid.Kd = tw.p[2];
              tw.step = 0;
              tw.epoch += 1;
              tw.sum_err = 0.;
              std::cout << "\nEpoch: " << tw.epoch << "\tMax epochs: " << tw.max_epochs << std::endl;
              reset_simulator(ws);
            }
            ++tw.step;
            steer_pid.Kp = tw.best_p[0];
            steer_pid.Ki = tw.best_p[1];
            steer_pid.Kd = tw.best_p[2];
          }

          steer_pid.UpdateError(cte);
          double normalized_error = steer_pid.TotalError();

          if (normalized_error > 1.) normalized_error = 1.;
          if (normalized_error < -1.) normalized_error = -1.;

          steer_value = -normalized_error;

          
          // DEBUG
          /*
          std::cout << "Diff CTE: " << steer_pid.d_error << std::endl;
          std::cout << "Int CTE: " << steer_pid.i_error << std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          */

          json msgJson;
          msgJson["steering_angle"] = steer_value;

          double throttle_error = (speed - target_speed) + 10. * fabs(steer_value) + 5. * fabs(cte);
          speed_pid.UpdateError(throttle_error);  
          double throttle_value = -speed_pid.TotalError();
          
          if (throttle_value > 1.) throttle_value = 1.;
          else if (throttle_value < -1.) throttle_value = -1.;

          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
