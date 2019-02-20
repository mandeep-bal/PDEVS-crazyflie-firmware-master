#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <assert.h>

// data structures
#include "../data_structures/message.hpp"

#include <algorithm>
#include <limits>

#include <boost/algorithm/string.hpp>
#include <boost/simulation.hpp>

using namespace std;

int main(int argc, char** argv) {

  assert(argc == 2);


    ifstream file(argv[1]);
    string str;
    string m_input_sensor;
    while (getline(file, str)){

      m_input_sensor += str;
      m_input_sensor.push_back('\n');
    }  

    cout << "model input sensor:" << endl;
    cout << m_input_sensor << endl;

    Message m1;
    Message m2;
    Message m3;
    Message m4;
    Message m5;
    Message m6;
    Message m7;
    Message m8;
    Message m9;
    Message m10;
    Message m11;
    Message m12;

    istringstream ss;
    ss.str(m_input_sensor);

    ss >> m1;
    ss >> m2;
    ss >> m3;
    //ss >> m4;
    //ss >> m5;
    //ss >> m6;
    //ss >> m7;
    //ss >> m8;
    //ss >> m9;
    //ss >> m10;
    //ss >> m11;
    //ss >> m12;

    cout << m1 << endl << endl;
    cout << m2 << endl << endl;
    cout << m3 << endl << endl;
    //cout << m4 << endl;
    //cout << m5 << endl;
    //cout << m6 << endl;
    //cout << m7 << endl;
    //cout << m8 << endl;
    //cout << m9 << endl;
    //cout << m10 << endl;
    //cout << m11 << endl;
    //cout << m12 << endl;



  return 0;
}