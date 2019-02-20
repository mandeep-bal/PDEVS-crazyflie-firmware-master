CC=g++-4.9
CFLAGS=-std=c++11
INCLUDEBOOST=-I ../../boost_1_57_0
INCLUDEBCDPP=-I ../../cdboost/include

all: sensorController_test.o ../../data_structures/message.o sensfusion6.o
	$(CC) -g -o test sensorController_test.o ../../data_structures/message.o sensfusion6.o

sensorController_test.o: sensorController_test.cpp
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) sensorController_test.cpp -o sensorController_test.o

../../data_structures/message.o: 
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) ../../data_structures/message.cpp -o ../../data_structures/message.o

sensfusion6.o: sensfusion6.c
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) sensfusion6.c -o sensfusion6.o

clean:
	rm -f test *.o *~
	-for d in ../../data_structures; do (cd $$d; rm -f *.o *~ ); done
	-for d in ../../atomic_models; do (cd $$d; rm -f *.o *~ ); done