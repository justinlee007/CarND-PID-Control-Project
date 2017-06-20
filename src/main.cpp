#include <uWS/uWS.h>
#include "json.hpp"
#include "PidController.h"
#include "Twiddle.h"
#include "Tracker.h"

// for convenience
using json = nlohmann::json;

// Starting values derived from multiple twiddle runs
static const double START_KP = 0.15;
static const double START_KI = 0.0004;
static const double START_KD = 3;

static const double SPEED_MAX = 100.0;
static const double SPEED_MIN = 0.0;
static const double THROTTLE_CEIL = 1.0;
static const double THROTTLE_FLOOR = 0.45;

static const double HIGH_CTE_THRESHOLD = 1.0;
static const double THROTTLE_HIGH_CTE = 0.25;

static const int SAMPLE_SIZE = 100;
static const double MIN_TOLERANCE = 0.2;

static bool low_tps_ = false;
static bool achieved_tolerance_ = false;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned, else the empty string "" will be returned.
 */
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PidController pid;
  Twiddle twiddle;
  Tracker tracker;

  pid.init(START_KP, START_KI, START_KD);
  twiddle.init(START_KP, START_KI, START_KD);
  tracker.init(SAMPLE_SIZE);

  h.onMessage([&pid, &twiddle, &tracker](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // Get the steering value from the PID controller
          pid.updateError(cte);
          double steer_value = pid.totalError();

          double throttle;
          if (fabs(cte) > HIGH_CTE_THRESHOLD) {
            // Go slow when the cte is high
            throttle = THROTTLE_HIGH_CTE;
          } else {
            // Otherwise, use the inverse of the steering value as the throttle, with a max of 100
            throttle = fmin(1 / fabs(steer_value), SPEED_MAX);

            // Normalize the throttle value from [0, 100] to [0.45, 1.0]
            // normalized_x = ((ceil - floor) * (x - minimum))/(maximum - minimum) + floor
            throttle = ((THROTTLE_CEIL - THROTTLE_FLOOR) * (throttle - SPEED_MIN)) / (SPEED_MAX - SPEED_MIN) + THROTTLE_FLOOR;

            if (low_tps_) {
              throttle -= 0.2;
            }
          }

          // Twiddle the parameters until tolerance is met
          if (!achieved_tolerance_) {
            twiddle.incrementCount(cte);
            if (twiddle.getCount() == SAMPLE_SIZE) {
              std::vector<double> params = twiddle.updateParams();
              if (twiddle.getTolerance() < MIN_TOLERANCE) {
                achieved_tolerance_ = true;
              } else {
                pid.init(params[0], params[1], params[2]);
              }
              low_tps_ = (tracker.getAveTps() < 30);
            }
          }

          tracker.onMessageProcessed(cte, speed, throttle);

          // DEBUG
          // printf("cte=%.4f, steer_value=%.4f, throttle=%.4f\n", cte, steer_value, throttle);

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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

  // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else { // i guess this should be done more gracefully?
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
