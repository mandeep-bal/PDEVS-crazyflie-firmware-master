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

//sensorfusion
#include "sensfusion6.h"

#define BOOST_TEST_MODULE eulerTransform
#include <boost/test/included/unit_test.hpp>

using namespace boost;
using namespace boost::unit_test;
using namespace std;


BOOST_AUTO_TEST_CASE( eulerTransform_test_type ) {

 eulerTransform<BRITime, Message> eulerTransform_test;
  Message m1(MsgType::QS, 1, 2, 3, 4, 5, 6, 7);
  eulerTransform_test.external({m1}, BRITime(1,1));
  vector<Message> result = eulerTransform_test.out();
  BOOST_CHECK_EQUAL(result.front().type, MsgType::EULER_DATA);

}


BOOST_AUTO_TEST_CASE (eulerTransform_right_results){

  float roll;
  float pitch;
  float yaw;

  eulerTransform<BRITime, Message> eulerTransform_test;
  int a = 0;
  for(float i1 = 1; i1<50; i1 += 1.0){
    for(float i2 = 1; i2<50; i2 += 1.0){
      for(float i3 = 1; i3<50; i3 += 1.0){
        for(float i4 = 1; i4<50; i4 += 1.0){
          a = a + 1;
          Message m1(MsgType::QS, i1, i2, i3, i4, 1, 2, 3);
          auxiliar_test_sensfusion6GetEulerRPY(i1, i2, i3, i4);
          eulerTransform_test.external({m1}, BRITime(a,1));
          vector<Message> result_message_model = eulerTransform_test.out();
          eulerTransform_test.internal(); // en este test no chequeas nada de lo que sucede con la internal, entonces para que esta esta linea ? me parece bien que esté si luego chequeas que andubo bien.
          //LLamo a la internal para actualizar el ta del modelo y que no tire errores, se supone que cuando ta != INF tira assert. Es como simular el funcionamiento de DEVS
          sensfusion6GetEulerRPY(&roll, &pitch, &yaw);            
          BOOST_CHECK_EQUAL(result_message_model.size(), 1);
          Message model_message = result_message_model.front();         
          BOOST_CHECK(abs(model_message.eulerTransform.euler_roll - roll) < 1e-4);
          BOOST_CHECK(abs(model_message.eulerTransform.euler_pitch - pitch) < 1e-4);
          BOOST_CHECK(abs(model_message.eulerTransform.euler_yaw - yaw) < 1e-4);              
        }
      }
    }
  }
}