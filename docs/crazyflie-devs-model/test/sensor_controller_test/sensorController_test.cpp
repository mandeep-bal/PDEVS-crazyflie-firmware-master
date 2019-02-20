#include <iostream>
#include <chrono>
#include <utility>
#include <memory>
#include <vector>
#include <math.h>
#include <iomanip>

// data structures
#include "../../vendor/britime.hpp"
#include "../../data_structures/message.hpp"

// Boost simalator include
#include <boost/simulation.hpp>

// Atomic model
#include "../../atomic_models/eulerTransform.hpp"
#include "../../atomic_models/qsUpdater.hpp"

//sensorfusion
#include "sensfusion6.h"

#define BOOST_TEST_MODULE sensor_controller
#include <boost/test/included/unit_test.hpp>

using namespace boost;
using namespace boost::unit_test;
using namespace std;





BOOST_AUTO_TEST_CASE (eulerTransform_right_results_with_mahony){
  resetqi();
  float roll;
  float pitch;
  float yaw;

  eulerTransform<BRITime, Message> eulerTransform_test;
  qsUpdater<BRITime, Message> qsUpdaterMahony_test(1, QuaternionType::MAHONY);
  int a = 0;
  for(int i1 = 1; i1<10; i1++){
    for(int i2 = 1; i2<10; i2++){
     for(int i3 = 1; i3<10; i3++){
      for(int i4 = 1; i4<10; i4++){
        for(int i5 = 1; i5<10; i5++){
          for(int i6 = 1; i6<10; i6++){
            Message m1(MsgType::SENSOR_DATA_QS, i1, i2, i3, i4, i5, i6);
            a = a + 1;
            qsUpdaterMahony_test.external({m1}, BRITime(a,1));
            vector<Message> result_qs = qsUpdaterMahony_test.out();
            qsUpdaterMahony_test.internal();

            BOOST_CHECK_EQUAL(result_qs.size(), 1);
            Message qs_model_message = result_qs.front();
            auxiliar_test_sensfusion6GetEulerRPY(qs_model_message.qs.q0, qs_model_message.qs.q1, qs_model_message.qs.q2, qs_model_message.qs.q3);
            eulerTransform_test.external({result_qs}, BRITime(a,1));
            vector<Message> result_euler_model = eulerTransform_test.out();
            eulerTransform_test.internal();
            sensfusion6GetEulerRPY(&roll, &pitch, &yaw);            
            BOOST_CHECK_EQUAL(result_euler_model.size(), 1);
            Message model_message = result_euler_model.front();         
            BOOST_CHECK(abs(model_message.eulerTransform.euler_roll - roll) < 1e-4);
            BOOST_CHECK(abs(model_message.eulerTransform.euler_pitch - pitch) < 1e-4);
            BOOST_CHECK(abs(model_message.eulerTransform.euler_yaw - yaw) < 1e-4);             
          }
        }
      }
     }
    }
  }
}


BOOST_AUTO_TEST_CASE (eulerTransform_right_results_with_madwick){
  resetqi();
  float roll;
  float pitch;
  float yaw;

  eulerTransform<BRITime, Message> eulerTransform_test;
  qsUpdater<BRITime, Message> qsUpdaterMadwick_test(1, QuaternionType::MADWICK);
  int a = 0;
  for(int i1 = 1; i1<10; i1++){
    for(int i2 = 1; i2<10; i2++){
     for(int i3 = 1; i3<10; i3++){
      for(int i4 = 1; i4<10; i4++){
        for(int i5 = 1; i5<10; i5++){
          for(int i6 = 1; i6<10; i6++){
            Message m1(MsgType::SENSOR_DATA_QS, i1, i2, i3, i4, i5, i6);
            a = a + 1;
            qsUpdaterMadwick_test.external({m1}, BRITime(a,1));
            vector<Message> result_qs = qsUpdaterMadwick_test.out();
            qsUpdaterMadwick_test.internal();

            BOOST_CHECK_EQUAL(result_qs.size(), 1);
            Message qs_model_message = result_qs.front();
            auxiliar_test_sensfusion6GetEulerRPY(qs_model_message.qs.q0, qs_model_message.qs.q1, qs_model_message.qs.q2, qs_model_message.qs.q3);
            eulerTransform_test.external({result_qs}, BRITime(a,1));
            vector<Message> result_euler_model = eulerTransform_test.out();
            eulerTransform_test.internal();
            sensfusion6GetEulerRPY(&roll, &pitch, &yaw);            
            BOOST_CHECK_EQUAL(result_euler_model.size(), 1);
            Message model_message = result_euler_model.front();         
            BOOST_CHECK(abs(model_message.eulerTransform.euler_roll - roll) < 1e-4);
            BOOST_CHECK(abs(model_message.eulerTransform.euler_pitch - pitch) < 1e-4);
            BOOST_CHECK(abs(model_message.eulerTransform.euler_yaw - yaw) < 1e-4);             
          }
        }
      }
     }
    }
  }
}
