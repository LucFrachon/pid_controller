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

  PID steer_pid, speed_pid;  //steering and speed controllers
  
  double Kp, Ki, Kd;  //parameters for steering controller
  double dKp, dKi, dKd;  //parameter increments for steering controller
  
  //Kp = 0.2; Ki = 0.0001; Kd = 3.;  //Initial values, uncomment when tuning twiddle
  Kp = 0.262871; Ki = 0.000143594; Kd = 3.50625;  //Optimized values for 40mph, comment out when tuning twiddle
  dKp = 0.02, dKi = 1.e-5, dKd = 0.3;  //For twiddle tuning

  steer_pid.Init(Kp, Ki, Kd);  //initialize steering PID
  speed_pid.Init(0.25, 0., 0.);  //initialize speed PID

  Twiddle tw;  //Twiddle class object
  tw.Init(Kp, Ki, Kd, dKp, dKi, dKd, 0.2, 3000, 30);  //Initialize Twiddle with initial parameters
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

          if (tw.tuning)  //if we are tuning parameters with Twiddle
          {
            if ((tw.step == 1) && (tw.directions[tw.p_i] == 1))  //increment parameter only before first simulator
            {                                                    //cycle, and if we were going in the up direction
              tw.p[tw.p_i] += tw.dp[tw.p_i];                     //with this parameter
            }

            tw.sum_err += cte * cte;  //increment cumulated square error each simulator cycle

            if ((tw.step == tw.max_steps || std::fabs(cte) > 2.2) && tw.sumScaledDp() > tw.tolerance
              && tw.epoch <= tw.max_epochs) //Max steps reached, vehicle left road, 
                                            //scaled sum of param increments not below tolerance
                                            //and max epochs since last improved not reached
            //Basically, execute these instructions when the simulator has run for the specified number
            //of steps.  
            {
              
              tw.avg_err = tw.sum_err / tw.max_steps;
              std::cout << "\nParameter index: " << tw.p_i;
              std::cout << "\nInitial p: " << tw.p[tw.p_i] << "\tInitial dp: " << tw.dp[tw.p_i];
              std::cout  << "\nAverage error: " << tw.avg_err << "\tBest error: " << tw.best_err;
              
              if (tw.avg_err < tw.best_err)  //if improved average error
              {
                tw.best_err = tw.avg_err;
                //keep track of best parameters
                tw.best_p[0] = tw.p[0];
                tw.best_p[1] = tw.p[1];
                tw.best_p[2] = tw.p[2];
                tw.epoch = 0;  //reset epoch because we just found better parameters
                tw.directions[tw.p_i] = 1;
                
                tw.updateDp(1 + tw.dp_factor);  //increase dp
                std::cout << "\nNew dp: " << tw.dp[tw.p_i];
                tw.p[tw.p_i] += tw.dp[tw.p_i];  //increment p
                std::cout << "\nNew p: " << tw.p[tw.p_i];
                
                tw.p_i = (tw.p_i + 1) % 3;  //move on to the next parameter
              }
              else if (1 == tw.directions[tw.p_i])  //if error not improved, and we were going up
              {
                tw.p[tw.p_i] -= 2 * tw.dp[tw.p_i];  //decrease p[i] twice because we increased it once before
                std::cout << "\nNew p: " << tw.p[tw.p_i];
                tw.directions[tw.p_i] = -1;  //we are now going down
              }
              else  //error not improve, and we were going down  
              {
                tw.p[tw.p_i] += tw.dp[tw.p_i];  //decrement p[i] to get to the last best value
                std::cout << "\nNew p: " << tw.p[tw.p_i];
                tw.updateDp(1 - tw.dp_factor);  //decrease dp
                std::cout << "\nNew dp: " << tw.dp[tw.p_i];
                tw.directions[tw.p_i] = 1;  //we will now go up again but in smaller increments
                std::cout << "\nNew direction for p: " << tw.directions[tw.p_i];
                tw.p_i = (tw.p_i + 1) % 3;  //move on to the next parameter
              }
              
              std::cout << "\nCurrent parameters:\n"
                << tw.p[0] << "\t" << tw.p[1] << "\t" << tw.p[2] << std::endl;
              std::cout << "\nCurrent best parameters:\n"
                << tw.best_p[0] << "\t" << tw.best_p[1] << "\t" << tw.best_p[2] << std::endl << std::endl;

              //set pid parameters to latest twiddle values
              steer_pid.Kp = tw.p[0];
              steer_pid.Ki = tw.p[1];
              steer_pid.Kd = tw.p[2];

              tw.step = 0; //reset step
              tw.sum_err = 0.;  //reset cumulative error
              tw.epoch += 1;  //increment epoch
              std::cout << "\nEpoch: " << tw.epoch << "\tMax epochs: " << tw.max_epochs << std::endl;
              reset_simulator(ws);  //reset simulator
            }

            ++tw.step;  //increment step

            //set pid parameters to best recorded parameters
            steer_pid.Kp = tw.best_p[0];
            steer_pid.Ki = tw.best_p[1];
            steer_pid.Kd = tw.best_p[2];
          }


          //use PID controller to compute steering angle
          steer_pid.UpdateError(cte);
          double normalized_error = steer_pid.TotalError();

          //normalize steering angle between -1 and +1          
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

          //use speed P controller to compute throttle/brake input
          double throttle_error = (speed - target_speed) + 10. * fabs(steer_value) + 5. * fabs(cte);
          speed_pid.UpdateError(throttle_error);  
          double throttle_value = -speed_pid.TotalError();
          
          //normalize throtte value between -1 and +1
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
