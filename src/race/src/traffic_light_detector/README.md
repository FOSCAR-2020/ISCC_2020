* run cmd

# comfile
g++ -o main main.cpp $(pkg-config --libs --cflags opencv)

# run
./main

