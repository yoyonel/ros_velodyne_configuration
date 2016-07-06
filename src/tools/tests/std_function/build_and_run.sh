#g++-4.8 -std=c++11  -O2 -Wall -Wextra -pedantic -pthread -pedantic-errors $1 -lm  && ./a.out
g++-4.8 -std=c++11 std_function.cpp && ./a.out
