REFERENCES = \
	-lm \
	-lfann
	
all: default

default: trainSegmentation.cpp
	g++ -o trainSegmentation trainSegmentation.cpp $(REFERENCES)

result: createUVDiagram.cpp
	g++ -o createUVDiagram createUVDiagram.cpp $(REFERENCES)

clean:
	rm -f trainSegmentation
