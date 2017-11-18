#include <iostream>
#include <fstream>
#include <sstream>
#include <uWS/uWS.h>
#include <math.h>
#include <chrono>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"

using namespace std;

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

int main(int argc, char* argv[])
{
  uWS::Hub h;

  // open a file
  ofstream outfile;

  // Initialize the pid variables for steer
  double Kp_st, Ki_st, Kd_st;
  if (argc >= 4){  // Read K values
    Kp_st = atof(argv[1]);
    Ki_st = atof(argv[2]);
    Kd_st = atof(argv[3]);
  }
  else {
    Kp_st = 0.3566;
    Ki_st = 0.0624;
    Kd_st = 0.1575;
  }
  int use_twiddle = 0;
  if (argc == 6){  // Open file and write first line
    use_twiddle = atoi(argv[4]);
    outfile.open(argv[5],ofstream::out);
    outfile << "Kp" << ", ";
    outfile << "Ki" << ", ";
    outfile << "Kd" << ", ";
    outfile << "Error" << endl;
  }

  // Twiddle parameters
  int n_steps = 2000;         // nb time steps for each twiddle trial
  int n_init_step_rem = 100;  // nb of initial steps to remove before starting to acc error
  double dKp = 0.2;          // initial Kp increment
  double dKi = 0.05;        // initial Ki increment
  double dKd = 0.2;          // initial Kd increment
  double tol = 0.005;        // threshold on sum(dK) for stopping twiddle

  Twiddle twiddle_st;
  if (use_twiddle) {
    cout << "Use TWIDDLE "<< endl;
    twiddle_st.Init(n_steps, dKp, dKi, dKd, tol, n_init_step_rem);
  }
  PID pid_steer;

  pid_steer.Init(Kp_st,Ki_st,Kd_st,twiddle_st);

  // Initialize the pid variables for speed
  double Kp_sp = 0.3;
  double Ki_sp = 0.0;
  double Kd_sp = 0.0;
  Twiddle twiddle_sp;  // no twiddle for speed
  PID pid_speed;

  pid_speed.Init(Kp_sp,Ki_sp,Kd_sp,twiddle_sp);

  std::cout << "PID steer: Kp: " << pid_steer.Kp_ << " Ki: " << pid_steer.Ki_ << " Kd: " << pid_steer.Kd_ << endl;
  std::cout << "PID speed: Kp: " << pid_speed.Kp_ << " Ki: " << pid_speed.Ki_ << " Kd: " << pid_speed.Kd_ << endl;

  h.onMessage([&pid_steer, &pid_speed, &outfile](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle;
          double speed_cmd = 40;      // PID speed command
          double cte_thr = 6.0;       // cte threshold for simulator reset

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // compute timestep
          long long msSinceEpoch = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();  // current time tag
          long dt_ms = msSinceEpoch-pid_steer.msSinceEpochPrev_;
          double dt = (double)dt_ms/1000;
          pid_steer.msSinceEpochPrev_ = msSinceEpoch;

          // compute steering value using PID controller
          pid_steer.UpdateError(cte, dt);
          steer_value = pid_steer.TotalError();
          // keep it in range [-1;1]
          if (steer_value>1) {
            steer_value = 1;
          } else if (steer_value<-1) {
            steer_value = -1;
          }
          // Compute cumulated squared error
          pid_steer.SSE(cte);

          // compute throttle value using PID controller
          double speed_error = speed - speed_cmd;
          pid_speed.UpdateError(speed_error, dt);
          throttle = pid_speed.TotalError();

          //throttle = 0.3;
          // Compute cumulated squared error
          pid_speed.SSE(speed_error);
          // DEBUG
          if (false) {
            std::cout << pid_steer.iter_ << ". CTE: " << cte << " Steering Value: " << steer_value << " dt (s): " << (double)dt ;
            std::cout << "/ Speed: " << speed << " Throttle Value: " << throttle << std::endl;
          }
          json msgJson;

          // reset simulator is CTE is greater than threshold
          bool flag_reset_oot = fabs(cte)>cte_thr;
          if (flag_reset_oot){
             cout << "cte overshoot -> reset simulator"<< endl;
          }
          // APPLY TWIDDLE ON STEER PID CONTROLLER
          // returns a bool to reset simulator after n_steps time steps
          // automatic stop of twiddle when tolerance criteria is reached
          bool flag_reset_twid;
          if (pid_steer.twiddle_.is_used_) {
            flag_reset_twid =  pid_steer.Process_twiddle(flag_reset_oot);
          }
          else {
            flag_reset_twid = false;
          }
          bool flat_reset_sim =  flag_reset_oot || flag_reset_twid;

          if (flat_reset_sim==false){  // NO RESET
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else if (flat_reset_sim==true) {  // RESET
            // copy results
            //cout << "Reset simulator" << endl;
            if (outfile.is_open()){
              outfile << fixed << setprecision(3) << pid_steer.Kp_ << ", ";
              outfile << fixed << setprecision(3) << pid_steer.Ki_ << ", ";
              outfile << fixed << setprecision(3) << pid_steer.Kd_ << ", ";
              outfile << fixed << setprecision(3) << pid_steer.error_ <<  endl;
            }
            //reinitialize controller
            pid_steer.is_initialized_ = false;
            pid_speed.is_initialized_ = false;
            //cout << "mark" << endl;
            // reset simulator
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
           }
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
    //std::cout << "Connected!!!" << std::endl;
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
  if (outfile.is_open()){
    outfile.close();
  }
}
