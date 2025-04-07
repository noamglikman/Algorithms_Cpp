
C = g++
CFLAGS = -Wall -Wextra -pedantic -std=c++11 -g

# Targets
TARGET = main
TEST_TARGET = test
# Files
OBJS = main.o Queue.o graph.o Algorithms.o UnionFind.o 
TEST_OBJS = test.o Queue.o graph.o Algorithms.o UnionFind.o
# Default rule
all: $(TARGET) $(TEST_TARGET)

# Linking
$(TARGET): $(OBJS)
	$(C) $(CFLAGS) -o $(TARGET) $(OBJS)
$(TEST_TARGET): $(TEST_OBJS)
	$(C) $(CFLAGS) -o $(TEST_TARGET) $(TEST_OBJS)
# Compile each .cpp to .o
main.o: main.cpp graph.h Queue.h Algorithms.h UnionFind.h 
	$(C) $(CFLAGS) -c main.cpp 
test.o: test.cpp graph.h Queue.h Algorithms.h UnionFind.h 
	$(C) $(CFLAGS) -c test.cpp
Queue.o: Queue.cpp Queue.h
	$(C) $(CFLAGS) -c Queue.cpp
graph.o: graph.cpp graph.h
	$(C) $(CFLAGS) -c graph.cpp

Algorithms.o: Algorithms.cpp Algorithms.h graph.h Queue.h UnionFind.h
	$(C) $(CFLAGS) -c Algorithms.cpp
UnionFind.o: UnionFind.cpp UnionFind.h graph.h
	$(C) $(CFLAGS) -c UnionFind.cpp

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET) $(TEST_OBJS) $(TEST_TARGET)
run_tests: $(TEST_TARGET)
	./$(TEST_TARGET)
valgrind: $(TARGET)
	valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes ./$(TARGET)