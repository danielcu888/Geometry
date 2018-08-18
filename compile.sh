#! /bin/bash -

g++ -o main main.cpp --std=c++11 -I/usr/local/boost_installed/include -lgtest -lgmock

exit $?