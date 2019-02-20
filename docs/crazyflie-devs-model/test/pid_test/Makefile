CC=g++-4.9
CFLAGS=-std=c++11
INCLUDEBOOST=-I ../../boost_1_57_0
INCLUDEBCDPP=-I ../../cdboost/include

all:
	$(CC) -g $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) pid_test.cpp controller/pid.c ../../data_structures/message.cpp -o test

clean:
	rm -f test *.o *~
	-for d in ../../data_structures; do (cd $$d; rm -f *.o *~ ); done
	-for d in ../../atomic_models; do (cd $$d; rm -f *.o *~ ); done