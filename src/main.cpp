#include <math.h>
#include <iostream>
#include <string>

#ifdef UWS_VCPKG
	// On windows, using the latest uwebsockets library
	#include <uwebsockets/App.h>
	#include "json_300.hpp"

#else
	// When the Udacity version of the uwebsockets library is used
	#include <uWS/uWS.h>
	#include "json.hpp"
#endif 

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
PIDTRAINER* pt = nullptr;

#ifdef USE_TRAINING
	double optimal_speed = 75;
#else 
	double optimal_speed = 50;
#endif 

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

// initialize the PID controllers and if configured also init. the PIDTRAINER
void init(int argc, char** argv, PID& pid, PID& pid_throttle)
{
	double p1, d1, i1;

	// the previously fine-tuned, best PID coefficients
//    p1 = 0.168901;
//    i1 = 0.00517276;
//    d1 = 3.87919;
	p1 = 0.15;
	i1 = 0.006;
	d1 = 4;

#ifdef USE_TRAINING
	double de1, de2, de3;
	if (argc == 7)
	{
		p1 = atof(argv[1]);
		i1 = atof(argv[2]);
		d1 = atof(argv[3]);
		de1 = atof(argv[4]);
		de2 = atof(argv[5]);
		de3 = atof(argv[6]);
	}

	pt = new PIDTRAINER(&pid, 1000, p1, i1, d1, de1, de2, de3);
#endif 

	pid.Init(p1, i1, d1);
	pid_throttle.Init(9999, 0, 0);
}

// the logic which uses the 2 PID controllers to control the new steer_value and throttle
void logic( PID &pid, PID &pid_throttle, double cte, double speed, double angle, double& steer_value, double& throttle) 
{
	pid.UpdateError(cte, speed, angle);
	steer_value = pid.TotalError();
	steer_value = min(steer_value, 1.0);
	steer_value = max(steer_value, -1.0);
	pid_throttle.UpdateError(speed - optimal_speed, speed, angle);
	throttle = pid_throttle.TotalError();
	throttle = min(throttle, 0.8);
	throttle = max(throttle, 0.0);
};

// process an incoming websocket message
// It contains the logic which restarts the simulation when a run is finished.
std::string process_message(const char* data, size_t length, PID& pid, PID& pid_throttle)
{
	std::string msg;
	if (length && length > 2 && data[0] == '4' && data[1] == '2') {

		if (!pt)
		{
			if (pid.samplenum == pt->target_samplenum) 
			{
				pt->ready();
				pid.samplenum = 0;
				msg = "42[\"reset\",{}]";
				return msg;
			}
		}

		auto s = hasData(string(data).substr(0, length));

		if (s != "") {
			auto j = json::parse(s);

			string event = j[0].get<string>();

			if (event == "telemetry") {
				// j[1] is the data JSON object
				double cte = std::stod(j[1]["cte"].get<string>());
				double speed = std::stod(j[1]["speed"].get<string>());
				double angle = std::stod(j[1]["steering_angle"].get<string>());
				double steer_value, throttle;

				logic(pid, pid_throttle, cte, speed, angle, steer_value, throttle);

				// DEBUG
				std::cout << "CTE: " << cte << " Steering Value: " << steer_value
					<< std::endl;

				json msgJson;
				msgJson["steering_angle"] = steer_value;
				msgJson["throttle"] = throttle;
				msg = "42[\"steer\"," + msgJson.dump() + "]";
				std::cout << msg << std::endl;
			}  // end "telemetry" if
		}
		else {
			// Manual driving
			msg = "42[\"manual\",{}]";
		}
	}  // end websocket message if

	return msg;
}

#ifndef UWS_VCPKG

int main() {
  uWS::Hub h;

  PID pid;
  PID pid_throttle;

  init(argc, argv, pid, pid_throttle);

  h.onMessage([&pid, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
	  auto msg = process_message(data, length, pid, pid_throttle);
	  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);	  
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

	init(argc, argv, pid, pid_throttle);

	struct PerSocketData {
        int something;
	};

	int port = 4567;

	uWS::App::WebSocketBehavior b;
    b.maxPayloadLength = 16 * 1024 * 1024;
	b.open = [](auto* ws) {
		std::cout << "Connected!!!" << std::endl;
	};
	b.close = [](auto* ws, int /*code*/, std::string_view /*message*/) {
		ws->close();
		std::cout << "Disconnected" << std::endl;
	};


    b.message = [&pid, &pid_throttle](auto* ws, std::string_view message, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
		size_t length = message.length();
		const char* data = message.data();
		auto msg = process_message(data, length, pid, pid_throttle);
		ws->send(msg, uWS::OpCode::TEXT);
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
