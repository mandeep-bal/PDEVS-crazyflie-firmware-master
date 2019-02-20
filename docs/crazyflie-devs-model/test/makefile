CC=g++-4.9
CFLAGS=-std=c++11
INCLUDEBOOST=-I ../boost_1_57_0
INCLUDEBCDPP=-I ../cdboost/include

all: compiling_test.o ../data_structures/message.o
	$(CC) -g -o compiling_test compiling_test.o ../data_structures/message.o

compiling_test.o: compiling_test.cpp
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) compiling_test.cpp -o compiling_test.o

../data_structures/message.o: 
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) ../data_structures/message.cpp -o ../data_structures/message.o

clean:
	rm -f compiling_test *.o *~
	-for d in ../data_structures; do (cd $$d; rm -f *.o *~ ); done
	-for d in ../atomic_models; do (cd $$d; rm -f *.o *~ ); done
	-for d in ../vendor; do (cd $$d; rm -f *.o *~ ); done