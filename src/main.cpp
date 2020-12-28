#include <math.h>

#ifdef UWS_VCPKG
#include <uwebsockets/App.h>
#else
#include <uWS/uWS.h>
#endif 

#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::min;
using std::max;
using std::endl;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

#ifndef UWS_VCPKG

int main() {
  uWS::Hub h;

  PID pid;
	PID pid_throttle;

    double p1, d1, i1, de1, de2, de3;

    // the previously fine-tuned, best PID coefficients
//    p1 = 0.168901;
//    i1 = 0.00517276;
//    d1 = 3.87919;
	p1 = 0.15;
	i1 = 0.006;
	d1 = 4;

#ifdef USE_TRAINING
    if (argc == 7)
    {
        p1 = atof(argv[1]);
		i1 = atof(argv[2]);
		d1 = atof(argv[3]);
		de1 = atof(argv[4]);
		de2 = atof(argv[5]);
		de3 = atof(argv[6]);
    }

	PIDTRAINER pt(&pid, 1000, p1, i1, d1, de1, de2, de3);
	double optimal_speed = 75;

#else 
    void* pt = nullptr;
	double optimal_speed = 50;
#endif 

    pid.Init(p1, i1, d1);              
    pid_throttle.Init(9999, 0, 0);

  h.onMessage([&pt, &pid, &pid_throttle, &optimal_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
#ifdef USE_TRAINING
			if (pid.samplenum == pt.target_samplenum) {
                pt.ready();
				std::string msg = "42[\"reset\",{}]";
				//						std::string msg = "42[\"reset\",{}​​​​​]";
				ws->send(msg, uWS::OpCode::TEXT);
                pid.samplenum = 0;
                return;
			}
#endif 

      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;

                    pid.UpdateError(cte, speed, angle);
                    steer_value = pid.TotalError();
                    steer_value = min(steer_value, 1.0);
					steer_value = max(steer_value, -1.0);
                    pid_throttle.UpdateError(speed - optimal_speed, speed, angle);
                    double throttle = pid_throttle.TotalError();
					throttle = min(throttle, 0.80);
                    throttle = max(throttle, 0.0);

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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

#else 

int main(int argc, char **argv) {

    PID pid;
	PID pid_throttle;

    double p1, d1, i1, de1, de2, de3;

    // the previously fine-tuned, best PID coefficients
//    p1 = 0.168901;
//    i1 = 0.00517276;
//    d1 = 3.87919;
	p1 = 0.15;
	i1 = 0.006;
	d1 = 4;

#ifdef USE_TRAINING
    if (argc == 7)
    {
        p1 = atof(argv[1]);
		i1 = atof(argv[2]);
		d1 = atof(argv[3]);
		de1 = atof(argv[4]);
		de2 = atof(argv[5]);
		de3 = atof(argv[6]);
    }

	PIDTRAINER pt(&pid, 1000, p1, i1, d1, de1, de2, de3);
	double optimal_speed = 75;

#else 
    void* pt = nullptr;
	double optimal_speed = 50;
#endif 

    pid.Init(p1, i1, d1);              
    pid_throttle.Init(9999, 0, 0);

	struct PerSocketData {
        int something;
	};

	int port = 4567;

	uWS::App::WebSocketBehavior b;
    b.compression = uWS::CompressOptions::SHARED_COMPRESSOR;
    b.maxPayloadLength = 16 * 1024 * 1024;
	b.open = [](auto* ws) {
		std::cout << "Connected!!!" << std::endl;
	};
	b.close = [](auto* ws, int /*code*/, std::string_view /*message*/) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	};
    b.upgrade = [](auto* res, auto* req, auto* context) {
        res->template upgrade<PerSocketData>({
            /* We initialize PerSocketData struct here */
                
			}, req->getHeader("sec-websocket-key"),
			req->getHeader("sec-websocket-protocol"),
			req->getHeader("sec-websocket-extensions"),
			context);
		int alma = 0;
    };


    b.message = [&pt, &pid, &pid_throttle, &optimal_speed](auto* ws, std::string_view message, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
		size_t length = message.length();
		const char* data = message.data();
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

#ifdef USE_TRAINING
			if (pid.samplenum == pt.target_samplenum) {
                pt.ready();
				std::string msg = "42[\"reset\",{}]";
				//						std::string msg = "42[\"reset\",{}​​​​​]";
				ws->send(msg, uWS::OpCode::TEXT);
                pid.samplenum = 0;
                return;
			}
#endif 

            auto s = hasData(string(data).substr(0, length));

            if (s != "") {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<string>());
                    double speed = std::stod(j[1]["speed"].get<string>());
                    double angle = std::stod(j[1]["steering_angle"].get<string>());
                    double steer_value;

                    pid.UpdateError(cte, speed, angle);
                    steer_value = pid.TotalError();
                    steer_value = min(steer_value, 1.0);
					steer_value = max(steer_value, -1.0);
                    pid_throttle.UpdateError(speed - optimal_speed, speed, angle);
                    double throttle = pid_throttle.TotalError();
					throttle = min(throttle, 0.80);
                    throttle = max(throttle, 0.0);

					// DEBUG
                    std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                        << std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    ws->send(msg, uWS::OpCode::TEXT);
                }  // end "telemetry" if
            }
            else {
                // Manual driving
                string msg = "42[\"manual\",{}]";
                ws->send(msg, uWS::OpCode::TEXT);
            }
        }  // end websocket message if
    }; // end h.onMessage

    uWS::App().ws<PerSocketData>("/*", std::move(b)).listen("127.0.0.1", port, [port](auto* listen_socket) {
        if (listen_socket) {
            std::cout << "Listening on port " << port << std::endl;
        }
        else {
            std::cerr << "Failed to listen to port" << std::endl;
            exit(-1);
        }
    }).run();

}

#endif 
