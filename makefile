

make:
	mkdir -p .out/build
	gcc -o .out/build/example example.c -I./include -L./lib -lmingw32 -lSDL2main -lSDL2_test -lSDL2
	cp -f bin/SDL2.dll .out/build/SDL2.dll
