#include "Whill.h"


#include "../libaqua/SerialPort.h"
#include "../libaqua/Thread.h"

#include <exception>

namespace __whill__ {
	
	class PacketSizeOutOfRangeError : public std::exception {
	};

  class WhillImpl : public whill::Whill, public ssr::Thread {
  private:
    ssr::SerialPort serial;

    whill::Velocity2D current_vel;
    whill::Pose2D current_pose;
	whill::Joy joy;


#define PACK_MAX 128
	uint8_t packet_buffer[PACK_MAX];
  private:
    bool endflag;
    long interval_ms;

	double right_motor_angle;
	double left_motor_angle;

  private:
	  const uint8_t PROTOCOL_SIGN = 0xAF;

	  void sendData(const uint8_t* data, const size_t size) {
		  static const uint8_t header = PROTOCOL_SIGN;

		  uint8_t cs = 0;
		  cs ^= header;
		  serial.Write(&header, 1);
		  const uint8_t len = size + 1;
		  cs ^= len;
		  serial.Write(&len, 1);

		  for (int i = 0; i < size; i++) {
			  cs ^= data[i];
			  serial.Write(data + i, 1);
		  }
		  serial.Write(&cs, 1);
	  }

	  void startStreaming(const uint16_t interval_ms) {
		  const uint8_t payload[] = { 0x00,  //Start Sending Data
								0x01,  //Dat1(Speed profiles)
								(uint8_t)(interval_ms << 8 & 0xFF),
								(uint8_t)(interval_ms << 0 & 0xFF),
							    0x00};
		  sendData(payload, sizeof(payload));
	  }

	  void stopStreaming() {
		  const uint8_t payload[] = { 0x01 };
		  sendData(payload, sizeof(payload));
	  }

	  uint8_t read_char() {
		  uint8_t c ;
		  while (true) {
			  if (serial.GetSizeInRxBuffer() > 0) {
				  serial.Read(&c, 1);
				  return c;
			  }
		  }
	  }

	  void waitPacketStart() {
		  uint8_t c;
		  do {
			  c = read_char();
		  } while (c != PROTOCOL_SIGN);
		  uint8_t len = read_char();
		  if (len > PACK_MAX) return;// throw new PacketSizeOutOfRangeError();
		  packet_buffer[0] = PROTOCOL_SIGN;
		  for (uint32_t i = 0; i < len; i++) {
			  packet_buffer[i+1] = read_char();
		  }
	  }

	  void parsePacket() {
		  joy.y = (int8_t)packet_buffer[15];
		  joy.x = (int8_t)packet_buffer[14];

		  right_motor_angle = (double)((int16_t)((packet_buffer[18] << 8) + (packet_buffer[19])) * 0.001);
		  left_motor_angle = (double)((int16_t)((packet_buffer[20] << 8) + (packet_buffer[21])) * 0.001);

	  }

  public:

    WhillImpl(const std::string& filename, const int baudrate, const int interval_ms=100): serial(filename.c_str(), baudrate),
	right_motor_angle(0), left_motor_angle(0) {
		startStreaming(interval_ms);
    }

    ~WhillImpl() {
		stopStreaming();
	}

	virtual void setJoy(int x, int y) {

		unsigned char payload[] = { 0x03,
								   0x00,   // Enable Host control
								   (unsigned char)(char)(y),
								   (unsigned char)(char)(x) };
		sendData(payload, sizeof(payload));
	}
    virtual void setTargetVelocity(const whill::Velocity2D& vel) {
		int x, y;

		float desire_front_kmh = vel.vx * 3.6;                    // [m/s]   to [km/h]
		float desire_spin_spd = -TREAD_WIDTH * vel.va * 3.6 * 2; // [rad/s] to [km/h]

		
		y = desire_front_kmh * 100 / (vel.vx >= 0 ? FORWARD_MAX : BACKWARD_MAX);
		y = y > 100 ? 100 : y;
		y = y < -100 ? -100 : y;
		
		x = desire_spin_spd * 100 / ROTATE_MAX;
		x = x > 100 ? 100 : x;
		x = x < -100 ? -100 : x;

		uint8_t payload[] = { 0x08, // Experimental Command, Control with Low Jerk, Almost Const-Accel control
							   0x00,
							   (uint8_t)(int8_t)y,
							   (uint8_t)(int8_t)x};
		sendData(payload, sizeof(payload));
    }
    
    virtual whill::Velocity2D getCurrentVelocity() const {
      return current_vel;
    }
    
    virtual whill::Pose2D getCurrentPose() const {
      return current_pose;
    }
    
    virtual void setCurrentPose(const whill::Pose2D& pose) {
      current_pose = pose;
    }

    virtual void startAutoUpdate(const long interval_ms = 100) {
      this->interval_ms = interval_ms;
      endflag = false;
      Start();
    }
    
    virtual void stopAutoUpdate() {
      endflag = true;
      Join();
    }
    
    virtual void updateOnce() {
		try {
			waitPacketStart();
			parsePacket();
		}
		catch (std::exception& ex) {

		}
    }

    virtual void Run() {
      while(!endflag) {
	    updateOnce();
	    Thread::Sleep(interval_ms/10);
      }
    }


	virtual whill::Joy getJoy() const {
		return joy;
	}
  };


};
