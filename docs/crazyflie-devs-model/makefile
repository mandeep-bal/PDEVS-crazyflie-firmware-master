CC=g++-4.9
CFLAGS=-std=c++11
INCLUDEBOOST=-I boost_1_57_0
INCLUDEBCDPP=-I cdboost/include

all: main.o data_structures/message.o model_generator/modelGenerator.hpp
	$(CC) -g -o controller main.o data_structures/message.o 

main.o: main.cpp
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) main.cpp -o main.o

data_structures/message.o: 
	$(CC) -g -c $(CFLAGS) $(INCLUDEBOOST) $(INCLUDEBCDPP) data_structures/message.cpp -o data_structures/message.o



clean:
	rm -f controller *.o *~
	-for d in data_structures; do (cd $$d; rm -f *.o *~ ); done
	-for d in atomic_models; do (cd $$d; rm -f *.o *~ ); done
	-for d in ../vendor; do (cd $$d; rm -f *.o *~ ); done
