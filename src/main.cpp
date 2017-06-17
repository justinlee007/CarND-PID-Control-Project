#include <uWS/uWS.h>
#include "json.hpp"
#include "PidController.h"
#include "Twiddle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

static const double SPEED_MAX = 100.0;
static const double SPEED_MIN = 0.0;
static const double SPEED_CEIL = 1.0;
static const double SPEED_FLOOR = 0.4;

static const int SAMPLE_SIZE = 20;

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

  PidController pid;
  Twiddle myTwiddle;

  double Kp = 0.225;
  double Ki = 0.0004;
  double Kd = 4;

  vector<double> params{Kp, Ki, Kd};

  pid.init(Kp, Ki, Kd);
  myTwiddle.init(params);

  h.onMessage([&pid, &myTwiddle, &params](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          throttle = fmin(1 / fabs(steer_value), SPEED_MAX);

          // Normalize the throttle value to between 0.4 and 1.0
          // normalized_x = ((ceil - floor) * (x - minimum))/(maximum - minimum) + floor
          throttle = ((SPEED_CEIL - SPEED_FLOOR) * (throttle - SPEED_MIN)) / (SPEED_MAX - SPEED_MIN) + SPEED_FLOOR;

          // DEBUG
//          printf("CTE: %.4f, Steering Value: %.4f, Throttle: %.4f\n", cte, steer_value, throttle);

          myTwiddle.incrementCount(cte);
          if (myTwiddle.getCount() == SAMPLE_SIZE) {
            params = myTwiddle.updateParams();
            myTwiddle.resetCount();
//            pid.init(Kp_, Ki_, Kd_);
          }

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
