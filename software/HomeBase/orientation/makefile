default: main

canny: cannyEdge.cpp
	g++ -c cannyEdge.cpp

main: main.cpp
	g++ main.cpp navimage.cpp -lopencv_core -lopencv_calib3d -lopencv_imgproc -lopencv_highgui -lopencv_contrib -lopencv_gpu -lopencv_stitching -lopencv_video -lopencv_videostab -o prog
	./prog

clean:
	rm prog
