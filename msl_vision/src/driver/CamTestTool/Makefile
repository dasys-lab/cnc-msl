
LDFLAGS = -lstdc++ -L. -ldc1394 -L/usr/X11R6/lib -lXv -lXext -lX11 -lICE -lSM
# -lcv -lcvaux
# -lhighgui -lSM

OBJ = XVDisplay.o \
    ../dc1394camera.o \
    ../guppy.o \
    ../marlin.o \
    ../pike.o \
    ../imagingsource.o \
	../ps3eye.o \
	tester/DelayTester.o \
	../CameraQuickCam.o \
	Filter.o \
	FilterYUVQuickCamToYUV.o \
	FilterYUVToRGB.o



all: $(OBJ)
	#gcc -g guppy_test.cpp -I/usr/include/opencv  -o GuppyTest $(OBJ) $(LDFLAGS)
	#gcc -g cam_test.cpp -I/usr/include/opencv  -o CamTest $(OBJ) $(LDFLAGS)
	#gcc guppy_test.cpp -I/usr/include/opencv -L. -lcv -lcvaux -lhighgui -o GuppyTest
	#colorgcc -g cam_test.cpp -o CamTest $(OBJ) $(LDFLAGS)
	colorgcc -g usb_cam_test.cpp -o UsbCamTest $(OBJ) $(LDFLAGS)

%.o: %.cpp
	#gcc -Wall -g -c $< -I/usr/include/opencv $(LDFLAGS) -o $@
	colorgcc -Wall -g -c $< $(LDFLAGS) -o $@


clean:
	rm -f *.o
	rm -f CamTest
	rm -f ../*.o
