#include <uWS/uWS.h>
#include "json.hpp"
#include "PID.h"
#include "Twiddle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned, else the empty string "" will be returned.
 */
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  Twiddle twiddle;
  // TODO: Initialize the pid variable.

  // VectorXd Params = VectorXd(5);
  double Kp = 0.225;
  double Ki = 0.0004;
  double Kd = 4;
  // Params<<Kp,Ki,Kd,.2,4;
  // twiddle.init(0.8,100,set_speed,Params);
  pid.init(Kp, Ki, Kd);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          double throttle;

          pid.updateError(cte);

          steer_value = pid.totalError();

          // Use the inverse of the steering value as the throttle, with a max of 100
          throttle = fmin(1 / fabs(steer_value), 100);

          // Normalize the throttle value to between 0.4 and 1.0
          // normalized_x = ((ceil - floor) * (x - minimum))/(maximum - minimum) + floor
          throttle = ((1.0 - 0.4) * throttle) / 100 + 0.4;

          // DEBUG
          printf("CTE: %.4f, Steering Value: %.4f, Throttle: %.4f\n", cte, steer_value, throttle);
          // twiddle.incrementCount();
//          if (twiddle.getCount()==40){
//            Params = twiddle.updateParams();
//             Kp = Params[0];
//            Ki = Params[1];
//            Kd = Params[2];
//            pid.init(Kp, Ki, Kd);
//            twiddle_tune.resetCount();
//            cout<<"Max_Speed = "<<max_speed<<endl;
//          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else { // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
