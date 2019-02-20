#include <iostream>
#include <chrono>
#include <memory>

// data structures
#include "../vendor/britime.hpp"
#include "../data_structures/message.hpp"

// Boost simalator include
#include <boost/simulation.hpp>

// Atomic models
#include "../atomic_models/pid.hpp"
#include "../atomic_models/eulerTransform.hpp"
#include "../atomic_models/qsUpdater.hpp"
#include "../atomic_models/powerCalculator.hpp"
#include "../atomic_models/controler.hpp"
#include "../atomic_models/swich.hpp"

using namespace std;

int main() {

  pid<BRITime, Message> pid1(PIDId::PID_ANGLE_ROLL, 1, 1, 1, true, 1,1);
  eulerTransform<BRITime, Message> eulerTransform1;
  qsUpdater<BRITime, Message> qsUpdater1(1.0, QuaternionType::MADWICK);
  powerCalculator<BRITime, Message> powerCalculator1(Formation::QUAD_FORMATION_NORMAL);
  controler<BRITime, Message> controler1;
  swich<BRITime, Message> swich1;


  cout << "[compiling test] generating models." << endl;
  Message mP1(MsgType::PID_INPUT, PIDId::PID_ANGLE_ROLL, PIDCommand::CALCULATE, 1, 2);
  Message mET1(MsgType::QS, 4, 4, 1, 2, 1, 4, 2);
  Message mQS1(MsgType::SENSOR_DATA, 4, 4, 4, 5, 5, 5);
  Message mQS2(MsgType::SENSOR_DATA_QS, 4, 4, 4, 5, 5, 5);
  Message mPC1(MsgType::POWER_DATA, (int16_t)2, (int16_t)3, (int16_t)4, (int16_t)5);
  Message mC1(MsgType::COMMANDER_INPUT, (float)2, (float)3, (float)4, (float)5, CommInpType::ANGLE, CommInpType::ANGLE, CommInpType::RATE);
  Message mC2(MsgType::EULER_DATA, 2, 3, 4, 5, 6, 6);

  cout << "[compiling test] Runing external functions." << endl;
  pid1.external({mP1}, BRITime(1));
  eulerTransform1.external({mET1}, BRITime(1));
  qsUpdater1.external({mQS2}, BRITime(1));
  powerCalculator1.external({mPC1}, BRITime(1));
  controler1.external({mC1, mC2}, BRITime(1));
  swich1.external({mQS1},BRITime(1));

  cout << "[compiling test] Runing out functions." << endl;
  pid1.out();
  eulerTransform1.out();
  qsUpdater1.out();
  powerCalculator1.out();
  controler1.out();
  swich1.out();

  cout << "[compiling test] Runing internal functions." << endl;
  pid1.internal();
  eulerTransform1.internal();
  qsUpdater1.internal();
  powerCalculator1.internal();
  controler1.internal();
  swich1.internal();

  cout << "[compiling test] Runing advance functions." << endl;
  pid1.advance();
  eulerTransform1.advance();
  qsUpdater1.advance();
  powerCalculator1.advance();
  controler1.advance();
  swich1.advance();

  cout << "[compiling test] finished." << endl;

  return 0;
}