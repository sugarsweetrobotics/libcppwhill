#pragma once


namespace whill {

	class Joy {
	public:
		int8_t x;
		int8_t y;

	public:
		Joy() : x(0), y(0) {}
		Joy(int8_t x, int8_t y) : x(x), y(y) {}
		~Joy() {};

		Joy(const Joy& j) : x(j.x), y(j.y) {}
		Joy& operator=(const Joy& j) {
			x = j.x;
			y = j.y;
			return *this;
		}
	};

	class Pose2D {
	public:
		double x;
		double y;
		double a;
	public:
		Pose2D(double x, double y, double a) : x(x), y(y), a(a) {}
		Pose2D() : x(0), y(0), a(0) {}
		~Pose2D() {}

		Pose2D(const Pose2D& p) : x(p.x), y(p.y), a(p.a) {}
		Pose2D& operator=(const Pose2D& p) {
			x = p.x; y = p.y; a = p.a; return *this;
		}

	};

	class Velocity2D {
	public:
		double vx;
		double vy;
		double va;
	public:
		Velocity2D(double vx, double vy, double va) : vx(vx), vy(vy), va(va) {}
		Velocity2D() : vx(0), vy(0), va(0) {}
		~Velocity2D() {}

		Velocity2D(const Velocity2D& p) : vx(p.vx), vy(p.vy), va(p.va) {}
		Velocity2D& operator=(const Velocity2D& p) {
			vx = p.vx; vy = p.vy; va = p.va; return *this;
		}
	};

	class Whill {
	private:

	public:
		~Whill() {}

		virtual void setTargetVelocity(const Velocity2D& vel) = 0;
		virtual Velocity2D getCurrentVelocity() const = 0;
		virtual Pose2D getCurrentPose() const = 0;
		virtual void setCurrentPose(const Pose2D& pose) = 0;

		virtual void startAutoUpdate(const long interval_ms = 100) = 0;
		virtual void stopAutoUpdate() = 0;
		virtual void updateOnce() = 0;

		virtual void setJoy(int x, int y) = 0;
		virtual Joy getJoy() const = 0;

		const float FORWARD_MAX = 6.0;     // [km/h]
		const float ROTATE_MAX = 3.5;      // [km/h]
		const float BACKWARD_MAX = 2.0;      // [km/h]
		const float TREAD_WIDTH = 0.248; // [m]
	};

};

#include "Whill.cpp"

namespace whill {

  inline Whill* createWhill(const std::string& filename) {
	  return new __whill__::WhillImpl(filename, 38400);
  }
};
