#include <iostream>
#include <chrono>
#include <utility>
#include <memory>
#include <vector>

// data structures
#include "../../vendor/britime.hpp"
#include "../../data_structures/message.hpp"

// Boost simalator include
#include <boost/simulation.hpp>

// Atomic model
#include "../../atomic_models/qsUpdater.hpp"

//sensorfusion
#include "sensfusion6.h"

#define BOOST_TEST_MODULE qsUpdater
#include <boost/test/included/unit_test.hpp>

using namespace boost;
using namespace boost::unit_test;

BOOST_AUTO_TEST_CASE( qsUpdater_test_type ) {

  qsUpdater<BRITime, Message> qsUpdaterMahony_test(1, QuaternionType::MAHONY);
  Message m1(MsgType::SENSOR_DATA_QS, 1, 2, 3, 4, 5, 6);
  qsUpdaterMahony_test.external({m1}, BRITime(1,1));
  vector<Message> result = qsUpdaterMahony_test.out();
  BOOST_CHECK_EQUAL(result.front().type, MsgType::QS);

}

BOOST_AUTO_TEST_CASE (qsUpdaterMahony_right_results){

  qsUpdater<BRITime, Message> qsUpdaterMahony_test(1, QuaternionType::MAHONY);
  resetqi();
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
            vector<Message> result_message_model = qsUpdaterMahony_test.out();
            qsUpdaterMahony_test.internal();
            vector<float> result_quad = sensfusion6UpdateQMAH(i1, i2, i3, i4, i5, i6, 1);
            BOOST_CHECK_EQUAL(result_message_model.size(), 1);
            Message qs_model_message = result_message_model.front();
            vector<float> result_model;
            result_model.push_back(qs_model_message.qs.q0);
            result_model.push_back(qs_model_message.qs.q1);
            result_model.push_back(qs_model_message.qs.q2);
            result_model.push_back(qs_model_message.qs.q3);
            for (int j = 1; j<result_model.size(); j++){
               BOOST_CHECK_EQUAL(result_quad[j], result_model[j]);
            }  
          }
        }
      }
     }
    }
  }
}

BOOST_AUTO_TEST_CASE (qsUpdaterMadwick_right_results){

  qsUpdater<BRITime, Message> qsUpdaterMadwick_test(1, QuaternionType::MADWICK);
  resetqi();
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
            vector<Message> result_message_model = qsUpdaterMadwick_test.out();
            qsUpdaterMadwick_test.internal();
            vector<float> result_quad = sensfusion6UpdateQMAD(i1, i2, i3, i4, i5, i6, 1);
            BOOST_CHECK_EQUAL(result_message_model.size(), 1);
            Message qs_model_message = result_message_model.front();
            vector<float> result_model;
            result_model.push_back(qs_model_message.qs.q0);
            result_model.push_back(qs_model_message.qs.q1);
            result_model.push_back(qs_model_message.qs.q2);
            result_model.push_back(qs_model_message.qs.q3);
            for (int j = 1; j<result_model.size(); j++){
               BOOST_CHECK_EQUAL(result_quad[j], result_model[j]);
            }  
          }
        }
      }
     }
    }
  }
}




