all: main.cpp rss.hpp
	g++ ./main.cpp -I./ -fsanitize=thread -O2 -g -std=c++17

run: all
	./a.out