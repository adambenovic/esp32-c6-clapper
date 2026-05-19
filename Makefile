IMAGE := espressif/idf:release-v5.4

PORT ?= /dev/ttyACM0

DOCKER := docker run --rm -v $(PWD):/project -w /project

.PHONY: build flash monitor clean

build:
	$(DOCKER) $(IMAGE) idf.py build

flash:
	$(DOCKER) --device $(PORT) $(IMAGE) idf.py -p $(PORT) flash

monitor:
	$(DOCKER) -it --device $(PORT) $(IMAGE) idf.py -p $(PORT) monitor

clean:
	$(DOCKER) $(IMAGE) idf.py fullclean
