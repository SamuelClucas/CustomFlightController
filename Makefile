Cxx := g++

UNAME_S := $(shell uname -s) 

ifeq ($(UNAME_S),Darwin)
libdir := -L/opt/homebrew/opt/glfw/lib -L/opt/homebrew/opt/boost/lib
incl := -I/opt/homebrew/opt/boost/include
lib := -lglfw -lboost_system
mac_frameworks := -framework OpenGL
else ifeq ($(UNAME_S),Linux)
libdir :=
incl :=
lib := -lglfw -lboost_system -lGL -lGLU -lm
mac_frameworks :=
endif

src := $(wildcard src/*.cpp)
inc := -Iinclude/
obj := $(src:.cpp=.o)
out := Observer

green := \\033[1;32m
blue := \\033[0;34m
black := \\033[0;30m

$(out): $(obj)
	@echo "Compiling the Observer..."
	@ $(Cxx) $^ $(inc) $(incl) $(libdir) $(lib) $(mac_frameworks) -o $@ -Wall -O2 
	@echo -e "$(green)The Observer is sleeping...$(black)"

src/%.o: src/%.cpp
	@echo -e "$(blue)Linking..."
	@echo $(Cxx) -c $(inc) $(incl) $< -o $@ -Wall -O2
	@$(Cxx) -c $(inc) $(incl) $< -o $@ -Wall -O2
	
run:
	./Observer

clean:
	rm -f $(out) $(obj)
