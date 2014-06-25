#
#CC	= /usr/local/arm/4.2.2-eabi/usr/bin/arm-linux-gcc
#LD	= /usr/local/arm/4.2.2-eabi/usr/bin/arm-linux-gcc
CC	= /opt/arm/usr/bin/arm-linux-gcc
#LD	= /opt/arm/usr/bin/arm-linux-gcc

BFLLIBPATH= .
EXE    = pboc_test

SOURCES := $(wildcard $(BFLLIBPATH)/*.c)

LIBCOBJECTS = $(patsubst %.c,%.o,$(SOURCES))
# -lpthread 
INCLUDE = -I .
CXXFLAGS = -W -g  $(INCLUDE) -O0 -L .   -lpthread 
CXXOPTIONS = -g -D "__DEBUG__" -O0 

all:$(LIBCOBJECTS)
	$(CC) $(LIBCOBJECTS) $(CXXFLAGS) -o $(EXE)
	cp $(EXE) /mnt/hgfs/wintolinux/lw_ftp/

$(LIBCOBJECTS): %.o: %.c
	$(CC) $(CXXFLAGS) -c $< -o $@


 
 

PHONY: clean
clean:
	find . -name "*.o" | xargs rm -f
	find ../Library -name "*.o" | xargs rm -f
	rm -f *.so.*
	rm -f *.a
	rm -f *.so
	rm -f core*
	rm -f rf_id_test

	

