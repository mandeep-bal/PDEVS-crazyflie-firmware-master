CC=g++-4.9
CFLAGS=-std=c++11
INCLUDEBOOST=-I ../../boost_1_57_0
INCLUDEBCDPP=-I ../../cdboost/include

all: qsUpdater_test.o ../../data_structures/message.o sensfusion6.o
	$(CC) -g -o test qsUpdater_test.o ../../data_structures/message.o sensfusion6.o

qsUpdater_test.o: qsUpdater_test.cpp
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) qsUpdater_test.cpp -o qsUpdater_test.o

../../data_structures/message.o: 
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) ../../data_structures/message.cpp -o ../../data_structures/message.o

sensfusion6.o: sensfusion6.c
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) sensfusion6.c -o sensfusion6.o

clean:
	rm -f test *.o *~
	-for d in ../../data_structures; do (cd $$d; rm -f *.o *~ ); done
	-for d in ../../atomic_models; do (cd $$d; rm -f *.o *~ ); done